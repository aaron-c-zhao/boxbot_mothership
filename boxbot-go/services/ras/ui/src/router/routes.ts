import { RouteConfig } from "vue-router";

export const LoginRoute: RouteConfig = {
    name: "login",
    path: "/login",
    component: () => import(/* webpackChunkName: "login" */ "../components/user/Login.vue"),
};

export const DashboardRoute: RouteConfig = {
    name: "Dashboard",
    path: "/",
    component: () => import(/* webpackChunkName: "dashboard" */ "../components/dashboard/Dashboard.vue"),
};

export const LogRoute: RouteConfig = {
    name: "Logs",
    path: "/logging",
    component: () => import(/* webpackChunkName: "logging" */ "../components/logging/Logs.vue"),
};

export const NotificationRoute: RouteConfig = {
    name: "Notifications",
    path: "/notifications",
    component: () => import(/* webpackChunkName: "notifications" */ "../components/notifications/Notifications.vue"),
};

export const NotFoundRoute: RouteConfig = {
    name: "NotFound",
    path: "*",
    component: () => import(/* webpackChunkName: "notifications" */ "../components/application/NotFound.vue"),
};

export const OperatingDashboardRoute: RouteConfig = {
    name: "OperationDashboard",
    path: "/operation",
    component: () => import(/* webpackChunkName: "dashboard" */ "../components/operating/Dashboard.vue"),
};

export const AppRoutes: RouteConfig[] = [DashboardRoute, LogRoute, NotificationRoute];

export const AppRoute: RouteConfig = {
    path: "/",
    component: () => import(/* webpackChunkName: "layout" */ "../components/application/Layout.vue"),
    children: [...AppRoutes, NotFoundRoute],
};

export const Routes: RouteConfig[] = [LoginRoute, OperatingDashboardRoute, AppRoute];
