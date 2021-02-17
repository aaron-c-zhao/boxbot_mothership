import Vue from "vue";
import Vuetify from "vuetify/lib";
import "vuetify/dist/vuetify.min.css";
// eslint-disable-next-line @typescript-eslint/ban-ts-ignore
// @ts-ignore
import DatetimePicker from "vuetify-datetime-picker";

Vue.use(Vuetify);
Vue.use(DatetimePicker);

export default new Vuetify({
    theme: {
        themes: {
            light: {
                primary: "#1867c0",
                secondary: "#5cbbf6",
                accent: "#005caf",
                error: "#FF5252",
                info: "#2196F3",
                success: "#4CAF50",
                warning: "#FFC107",
            },
        },
    },
    iconfont: "mdi",
});
