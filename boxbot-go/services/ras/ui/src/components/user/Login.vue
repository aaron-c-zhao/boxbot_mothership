<template>
    <v-container fill-height fluid>
        <v-row align-center justify-center>
            <v-col md4 sm8 xs12>
                <transition appear name="slide-fade" mode="in-out">
                    <v-card class="mx-auto pa-8" max-width="500px" outlined>
                        <v-row class="flex-column" align="center" justify="center">
                            <logo :website="true"></logo>
                            <h1 class="px-2 my-2" style="color: rgb(0, 113, 37)">BoxBot Maintenance</h1>
                            <v-form class="px-2 my-2" @submit.prevent="login">
                                <v-text-field
                                    v-model="password"
                                    color="primary"
                                    height="48"
                                    label="Password"
                                    autocomplete="current-password"
                                    type="password"
                                    :error-messages="errors"
                                    :hide-details="errors.length === 0"
                                />
                                <v-btn
                                    color="primary"
                                    block
                                    x-large
                                    class="text-capitalize px-2 mt-5"
                                    style="color: white"
                                    @click="login"
                                    >Login</v-btn
                                >
                            </v-form>
                        </v-row>
                    </v-card>
                </transition>
            </v-col>
        </v-row>
    </v-container>
</template>

<script lang="ts">
import { defineComponent, ref } from "@vue/composition-api";
import Logo from "../application/Logo.vue";
import { LoginState, loginState } from "@/state/userState";
import { DashboardRoute } from "@/router/routes";
import router from "@/router";

export default defineComponent({
    components: { Logo },
    setup(_, ctx) {
        const errors = ref<string[]>([]);
        const password = ref("");

        function login() {
            if (password.value.trim().length === 0) {
                errors.value = ["Password cannot be empty"];
            } else {
                // TODO(timanema): Actual login
                console.log(`password = ${password.value}`);
                loginState.state = LoginState.AUTHENTICATED;

                let target = ctx.root.$route.query.returnUri;
                if (typeof target !== "string") {
                    target = DashboardRoute.path;
                }

                console.log(`directing to ${target}`);
                router.push({ path: target });
            }
        }

        return {
            errors,
            password,
            login,
        };
    },
});
</script>

<style scoped lang="scss">
h1 {
    font-family: $alt-font;
    font-size: 1.6em;
}

.slide-fade-enter-active {
    transition: all 1s ease;
}
.slide-fade-leave-active {
    transition: all 0.2s ease;
}
.slide-fade-enter {
    transform: translateY(30px);
    opacity: 0;
}
.slide-fade-enter-to {
    opacity: 1;
}
.slide-fade-leave-to {
    opacity: 0;
}
</style>
