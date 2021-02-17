<template>
    <v-main
        app
        :class="state.error ? 'blink' : ''"
        :style="state.error ? 'color: white;' : 'background: white; color: rgb(173,16,46,255);'"
    >
        <v-app-bar v-if="!state.error" app flat dense class="v-bar--underline" color="rgb(0,0,0,0)">
            <v-img src="@/assets/boxbot-logo.png" class="shrink" max-width="35" />
            <v-toolbar-title class="ml-3"> {{ time }}</v-toolbar-title>
            <v-spacer />
        </v-app-bar>

        <v-container id="operationContainer" fill-height fluid>
            <v-row class="" justify="center" align="center">
                <v-col id="layoutContainer">
                    <v-progress-circular v-if="connecting" indeterminate size="70" />

                    <span v-else-if="connectingFailed">
                        <v-row justify="center">
                            <v-icon color="rgb(173, 16, 46)" x-large class="mr-2"> mdi-alert-decagram-outline</v-icon>
                            Connecting to BoxBot failed.
                        </v-row>
                        <v-row justify="center"> Is it powered on? </v-row>
                        <v-row justify="center" class="mt-5">
                            <v-btn outlined color="rgb(173,16,46)" @click="connect">Retry</v-btn>
                        </v-row>
                    </span>

                    <span v-else>
                        <v-row justify="center"> {{ state.message }} </v-row>
                        <v-row justify="center" style="font-size: x-large"> {{ state.hint }} </v-row>
                    </span>
                </v-col>
            </v-row>
        </v-container>
    </v-main>
</template>

<script lang="ts">
import { defineComponent, onMounted, onUnmounted, reactive, ref } from "@vue/composition-api";
import { format } from "date-fns";
import { DisplayMessage } from "@/components/operating/message";

export default defineComponent({
    setup() {
        const time = ref(format(new Date(), "HH:mm"));
        const state = reactive<DisplayMessage>({
            error: false,
            message: "",
            hint: "",
        });

        const connecting = ref(false);
        const connectingFailed = ref(false);

        function connect() {
            connecting.value = true;
            connectingFailed.value = false;

            const ws = new WebSocket("ws://localhost:8082/");

            ws.onerror = () => {
                connecting.value = false;
                connectingFailed.value = true;
            };

            ws.onopen = () => {
                connecting.value = false;
                state.error = false;
                state.message = "";
                state.hint = "";
            };

            ws.onclose = () => {
                connectingFailed.value = true;
            };

            ws.onmessage = (ev) => {
                const msg: DisplayMessage = JSON.parse(ev.data);

                state.error = msg.error;
                state.message = msg.message;
                state.hint = msg.hint;
            };
        }

        let timeInterval: any;
        onMounted(() => {
            timeInterval = setInterval(() => {
                time.value = format(new Date(), "HH:mm");
            }, 1000 * 5);

            connect();
        });

        onUnmounted(() => {
            clearInterval(timeInterval);
        });

        return {
            time,
            state,
            connecting,
            connectingFailed,

            connect,
        };
    },
});
</script>

<style lang="scss">
#operationContainer {
    font-size: xx-large;
}

@keyframes fade {
    from {
        background-color: rgb(240, 16, 46);
    }
    50% {
        background-color: rgb(173, 16, 46);
    }
    to {
        background-color: rgb(240, 16, 46);
    }
}

@-webkit-keyframes fade {
    from {
        background-color: rgb(240, 16, 46);
    }
    50% {
        background-color: rgb(173, 16, 46);
    }
    to {
        background-color: rgb(240, 16, 46);
    }
}

.blink {
    animation: fade 1500ms infinite;
    -webkit-animation: fade 1500ms infinite;
}
</style>
