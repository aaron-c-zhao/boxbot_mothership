<template>
    <div id="layoutDiv">
        <toolbar />
        <v-main app>
            <v-container>
                <v-row class="pt-5" justify="center">
                    <v-card outlined class="pa-2 full-height" style="width: 95%" max-width="1300px">
                        <v-container id="layoutContainer" class="pa-4" fluid>
                            <v-tabs>
                                <v-tab
                                    v-for="tab in tabs"
                                    :key="tab.path"
                                    class="tab ml-1"
                                    :exact="true"
                                    :to="tab.path"
                                    fixed-tabs
                                    @keyup.native.right="test"
                                >
                                    <v-badge
                                        v-if="tab.name === notificationName"
                                        color="info"
                                        :content="unreadNotifications.length"
                                        :value="unreadNotifications.length"
                                    >
                                        {{ tab.name }}
                                    </v-badge>
                                    <span v-else>
                                        {{ tab.name }}
                                    </span>
                                </v-tab>
                            </v-tabs>

                            <transition name="fade" mode="out-in">
                                <router-view />
                            </transition>
                        </v-container>
                    </v-card>
                </v-row>
            </v-container>
        </v-main>
    </div>
</template>

<script lang="ts">
import Toolbar from "./Toolbar.vue";
import { defineComponent } from "@vue/composition-api";
import Logo from "@/components/application/Logo.vue";
import { AppRoutes, NotificationRoute } from "@/router/routes";
import { unreadNotifications } from "@/state/notificationState";

export default defineComponent({
    setup() {
        return {
            test: function () {
                console.log("->");
            },
            notificationName: NotificationRoute.name,
            tabs: AppRoutes,
            unreadNotifications: unreadNotifications,
        };
    },
    components: {
        Logo,
        Toolbar,
    },
});
</script>

<style lang="scss">
#app {
    background-image: url(./bg.jpg);
    background-attachment: fixed;
    background-repeat: no-repeat;
    background-size: cover;
    background-position: center top;
}

.fade-enter-active {
    transition: all 0.4s ease;
}
.fade-leave-active {
    transition: all 0.2s ease;
}
.fade-enter {
    opacity: 0;
}
.fade-enter-to {
    opacity: 1;
}
.fade-leave-to {
    opacity: 0;
}
</style>
