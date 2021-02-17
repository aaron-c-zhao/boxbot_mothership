<template>
    <a v-if="website" class="text-decoration-none" href="https://heineken.com">
        <v-img src="@/assets/heineken-green.png" class="shrink" max-width="164" />
    </a>
    <router-link v-else to="/" class="d-inline-block">
        <v-img src="@/assets/heineken-white.png" class="shrink ml-1 mb-1" max-width="95" />
    </router-link>
</template>

<script lang="ts">
import { defineComponent } from "@vue/composition-api";

export default defineComponent({
    props: {
        website: {
            type: Boolean,
            default: false,
        },
    },
});
</script>
