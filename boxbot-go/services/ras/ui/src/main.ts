import "./plugins/composition-api";

import "@babel/polyfill";
import Vue from "vue";
import App from "./App.vue";
import "./registerServiceWorker";
import router from "./router";
import vuetify from "./plugins/vuetify";

import "@/styles/index.scss";

Vue.config.productionTip = false;

new Vue({
    router,
    vuetify,
    render: (h) => h(App),
}).$mount("#app");
