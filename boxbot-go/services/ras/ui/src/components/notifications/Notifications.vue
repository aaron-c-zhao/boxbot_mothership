<template>
    <span>
        <v-data-table
            :options.sync="options"
            :headers="headers"
            :items="notifications"
            item-key="id"
            :item-class="style"
            :footer-props="{ itemsPerPageOptions: [10, 50, 100, -1] }"
        >
            <template v-slot:item.timestamp="{ item }">
                <span>{{ new Date(item.timestamp).toLocaleString() }}</span>
            </template>
        </v-data-table>
    </span>
</template>

<script lang="ts">
import { defineComponent, onMounted, onUnmounted, ref } from "@vue/composition-api";
import { DataOptions } from "vuetify";
import { notifications, unreadNotifications } from "@/state/notificationState";
import { LogLevel } from "@/api/logs";

export default defineComponent({
    setup() {
        const options = ref<DataOptions>({
            groupBy: [],
            groupDesc: [],
            itemsPerPage: 10,
            multiSort: false,
            mustSort: false,
            page: 1,
            sortBy: [],
            sortDesc: [],
        });

        let interval: any;
        onMounted(() => {
            interval = setInterval(() => {
                unreadNotifications.value = [];
            }, 2000);
        });

        onUnmounted(() => {
            clearInterval(interval);
        });

        return {
            options,
            notifications: notifications,
            logLevels: Object.keys(LogLevel),
            headers: [
                { text: "Timestamp", value: "timestamp", sortable: false, width: "20%" },
                { text: "Message", value: "message", sortable: false, width: "80%" },
            ],
            style: function (e: NotificationData) {
                if (unreadNotifications.value.filter((m) => m.id == e.id).length >= 1) {
                    return "unread";
                }

                return "";
            },
        };
    },
});
</script>

<style lang="scss">
.unread {
    background-color: rgb(221, 243, 255);
}
</style>
