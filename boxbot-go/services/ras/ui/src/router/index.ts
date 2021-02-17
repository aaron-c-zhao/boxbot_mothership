import Vue from "vue";
import VueRouter from "vue-router";
import { Routes } from "@/router/routes";
import { checkAuth } from "@/router/guards/checkAuth";

Vue.use(VueRouter);

const router = new VueRouter({
    mode: "history",
    base: process.env.BASE_URL,
    routes: Routes,
});

router.beforeEach((to, from, next) => {
    checkAuth(to, from, next);
});

export default router;
