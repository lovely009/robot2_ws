import { createApp } from "vue";
import App from "./App.vue";
import { installTestHooks } from "./runtime";
import { pinia } from "./stores";
import "./styles/tokens.css";
import "./styles/panels.css";
import "./style.css";

installTestHooks();
if (!new URLSearchParams(location.search).has("test_modules")) {
  createApp(App).use(pinia).mount("#app");
}
