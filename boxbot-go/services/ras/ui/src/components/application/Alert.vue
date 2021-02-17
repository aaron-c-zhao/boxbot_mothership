<template>
    <v-snackbar
        v-model="alert"
        :color="alertState.type"
        outlined
        :timeout="5000"
        :top="true"
        :multi-line="true"
        class="text-center alert"
        transition="scale-transition"
    >
        {{ alertState.text }}

        <template v-slot:action="{ attrs }">
            <v-btn :color="alertState.type" text v-bind="attrs" @click="alert = false"> Close </v-btn>
        </template>
    </v-snackbar>
</template>

<script lang="ts">
import { computed, defineComponent } from "@vue/composition-api";
import { alertState } from "@/state/alertState";

export default defineComponent({
    setup() {
        const alert = computed({
            get: () => alertState.value.enabled,
            set(value) {
                alertState.value.enabled = value;
            },
        });

        return {
            alert,
            alertState,
        };
    },
});
</script>
