import json
import threading
import unittest
from io import BytesIO
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import server


class ServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.http = server.make_server(0, 'test-secret', 'https://api.openai.com/v1')
        cls.worker = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.worker.start()
        cls.url = f'http://127.0.0.1:{cls.http.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.worker.join()

    def post(self, data=None, origin=None, **headers):
        return urlopen(Request(self.url + '/api/session',
                               data=json.dumps(data if data is not None else {'sdp': 'v=0\r\n'}).encode(),
                               headers={'Origin': origin or self.url,
                                        'Content-Type': 'application/json', **headers}))

    def test_only_explicit_assets_are_served(self):
        with urlopen(self.url) as response:
            body = response.read()
            self.assertIn('开始对话'.encode(), body)
            self.assertNotIn(b'test-secret', body)
        with self.assertRaises(HTTPError) as error:
            urlopen(self.url + '/server.py')
        self.assertEqual(error.exception.code, 404)

    def test_foreign_origin_rejected(self):
        with self.assertRaises(HTTPError) as error:
            self.post(origin='https://example.com')
        self.assertEqual(error.exception.code, 403)

    def test_dns_rebinding_host_rejected(self):
        with self.assertRaises(HTTPError) as error:
            self.post(Host='evil.test')
        self.assertEqual(error.exception.code, 403)

    def test_bad_sdp_rejected(self):
        for data in ({'sdp': ''}, [], {'sdp': 1}):
            with self.subTest(data=data), self.assertRaises(HTTPError) as error:
                self.post(data)
            self.assertEqual(error.exception.code, 400)

    def test_large_request_rejected(self):
        with self.assertRaises(HTTPError) as error:
            self.post({'sdp': 'v=0' + 'x' * 65536})
        self.assertEqual(error.exception.code, 413)

    def test_live_contract_and_secret_isolation(self):
        result = {'session': {'id': 'live_test', 'secret': 'private'},
                  'transport': {'sdp': 'answer'}}
        with patch.object(server, 'urlopen') as upstream:
            upstream.return_value.__enter__.return_value = BytesIO(json.dumps(result).encode())
            with self.post() as response:
                received = json.load(response)
            request = upstream.call_args.args[0]
            self.assertEqual(request.full_url, 'https://api.openai.com/v1/live/sessions')
            payload = json.loads(request.data)
            self.assertEqual(payload['session']['model'], 'gpt-live-1')
            self.assertEqual(payload['transport']['type'], 'webrtc')
            self.assertEqual(request.headers['Authorization'], 'Bearer test-secret')
            self.assertEqual(received, {'session': {'id': 'live_test'}, 'transport': {'sdp': 'answer'}})

    def test_upstream_error_body_is_not_exposed(self):
        with patch.object(server, 'urlopen', side_effect=HTTPError('url', 403, 'denied', {}, BytesIO(b'test-secret'))):
            with self.assertRaises(HTTPError) as error:
                self.post()
            self.assertNotIn(b'test-secret', error.exception.read())

    def test_environment_credentials(self):
        with patch.dict(server.os.environ, {'OPENAI_API_KEY': 'env-secret'}, clear=True):
            self.assertEqual(server.credentials(), ('env-secret', 'https://api.openai.com/v1'))

    def test_page_key_overrides_default_without_changing_it(self):
        with patch.object(self.http, 'base_url', 'https://gateway.invalid/v1'), patch.object(server, 'urlopen') as upstream:
            upstream.return_value.__enter__.return_value = BytesIO(
                b'{"session":{"id":"live_test"},"transport":{"sdp":"answer"}}')
            with self.post({'sdp': 'v=0', 'api_key': 'sk-page-test'}) as response:
                self.assertNotIn(b'sk-page-test', response.read())
            request = upstream.call_args.args[0]
            self.assertEqual(request.headers['Authorization'], 'Bearer sk-page-test')
            self.assertEqual(request.full_url, 'https://api.openai.com/v1/live/sessions')
            self.assertEqual(self.http.api_key, 'test-secret')

    def test_invalid_page_keys_do_not_reach_upstream(self):
        with patch.object(server, 'urlopen') as upstream:
            for key in (None, 123, 'bad\r\nheader', '中文', 'a' * 513):
                with self.subTest(key=key), self.assertRaises(HTTPError) as error:
                    self.post({'sdp': 'v=0', 'api_key': key})
                self.assertEqual(error.exception.code, 400)
            upstream.assert_not_called()

    def test_missing_key_returns_actionable_error(self):
        with patch.object(self.http, 'api_key', ''), patch.object(server, 'urlopen') as upstream:
            with self.assertRaises(HTTPError) as error:
                self.post()
            self.assertIn('填写'.encode(), error.exception.read())
            upstream.assert_not_called()

    def test_no_local_key_is_allowed(self):
        with patch.dict(server.os.environ, {}, clear=True), patch.object(server.Path, 'exists', return_value=False):
            self.assertEqual(server.credentials(), ('', 'https://api.openai.com/v1'))


if __name__ == '__main__':
    unittest.main()
