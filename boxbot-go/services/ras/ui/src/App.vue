<template>
    <div id="app">
        <v-app>
            <alert />
            <transition name="fade" mode="out-in">
                <router-view id="container" />
            </transition>
        </v-app>
    </div>
</template>

<script lang="ts">
import Alert from "@/components/application/Alert.vue";
import { defineComponent } from "@vue/composition-api";

export default defineComponent({
    components: {
        Alert,
    },
});
</script>

<style lang="scss">
#app {
    font-family: $alt-font;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    text-align: center;
    color: #2c3e50;
}

html {
    overflow-y: auto !important;
    overflow-x: auto !important;
}

.fade-enter-active {
    transition: all 1s ease;
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
