package main

import (
	"archive/zip"
	"bytes"
	"os"
	"path/filepath"
	"testing"
)

func archiveWith(t *testing.T, name string) []byte {
	t.Helper()
	var data bytes.Buffer
	writer := zip.NewWriter(&data)
	file, err := writer.Create(name)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = file.Write([]byte("test")); err != nil {
		t.Fatal(err)
	}
	if err = writer.Close(); err != nil {
		t.Fatal(err)
	}
	return data.Bytes()
}

func TestUnpack(t *testing.T) {
	dir := t.TempDir()
	if err := unpack(archiveWith(t, "nested/app.txt"), dir); err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(filepath.Join(dir, "nested", "app.txt"))
	if err != nil || string(data) != "test" {
		t.Fatalf("bad extracted file: %v", err)
	}
}

func TestRejectUnsafePaths(t *testing.T) {
	for _, name := range []string{"../escape", "..\\escape", "/absolute", "C:/absolute"} {
		t.Run(name, func(t *testing.T) {
			if unpack(archiveWith(t, name), t.TempDir()) == nil {
				t.Fatal("unsafe path accepted")
			}
		})
	}
}

func TestPayloadContainsOnlyRuntimeAndApp(t *testing.T) {
	archive, err := zip.NewReader(bytes.NewReader(payload), int64(len(payload)))
	if err != nil {
		t.Fatal(err)
	}
	names := map[string]bool{}
	for _, file := range archive.File {
		names[file.Name] = true
	}
	for _, name := range []string{"python.exe", "server.py", "index.html", "app.js", "LICENSE.txt"} {
		if !names[name] {
			t.Errorf("missing %s", name)
		}
	}
	for _, name := range []string{"cc-switch.db", "config.toml", "auth.json", ".env"} {
		if names[name] {
			t.Errorf("unexpected private file: %s", name)
		}
	}
}
