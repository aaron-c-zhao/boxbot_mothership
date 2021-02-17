<template>
    <v-app-bar app flat class="v-bar--underline" color="rgb(0,0,0,0.6)">
        <logo />
        <v-spacer />
        <v-toolbar-title class="mr-2">BoxBot Maintenance</v-toolbar-title>

        <v-tooltip bottom>
            <template v-slot:activator="{ on, attrs }">
                <v-btn icon v-bind="attrs" v-on="on" @click="logout">
                    <v-icon color="white">mdi-logout-variant</v-icon>
                </v-btn>
            </template>
            <span>Logout</span>
        </v-tooltip>
    </v-app-bar>
</template>

<script lang="ts">
import { defineComponent } from "@vue/composition-api";
import Logo from "./Logo.vue";
import { LoginState, loginState } from "@/state/userState";
import router from "@/router";
import { LoginRoute } from "@/router/routes";

export default defineComponent({
    components: { Logo },
    setup() {
        return {
            logout: () => {
                // TODO(timanema): Actual logout
                loginState.state = LoginState.UNAUTHENTICATED;
                router.push(LoginRoute.path);
            },
        };
    },
});
</script>

<style scoped lang="scss">
.v-bar--underline {
    border-width: 0 0 thin 0;
    border-style: solid;
    border-bottom-color: #0000001f !important;
}

.v-toolbar__title {
    font-family: $alt-font;
    font-size: 1.6em;
    color: white;
}
</style>
