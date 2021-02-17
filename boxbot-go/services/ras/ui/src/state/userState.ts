import { reactive } from "@vue/composition-api";

export enum LoginState {
    AUTHENTICATED,
    UNAUTHENTICATED,
}

export const loginState = reactive<{
    state: LoginState;
}>({
    state: LoginState.AUTHENTICATED,
});
