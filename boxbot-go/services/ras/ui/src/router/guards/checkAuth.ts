import { DashboardRoute, LoginRoute } from "@/router/routes";
import { NavigationGuard } from "vue-router";
import { loginState, LoginState } from "@/state/userState";

export const checkAuth: NavigationGuard = (to, from, next) => {
    if (to.path !== LoginRoute.path && loginState.state === LoginState.UNAUTHENTICATED) {
        next({
            path: LoginRoute.path,
            query: {
                returnUri: to.fullPath,
            },
            replace: true,
        });
    } else if (to.path === LoginRoute.path && loginState.state === LoginState.AUTHENTICATED) {
        next(DashboardRoute.path);
    } else {
        next();
    }
};
