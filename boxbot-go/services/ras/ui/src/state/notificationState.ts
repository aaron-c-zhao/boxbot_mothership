import { ref } from "@vue/composition-api";
import { NotificationData } from "@/api/notifications";

export const unreadNotifications = ref<NotificationData[]>([
    {
        id: 1,
        timestamp: new Date(),
        message: "BoxBot is starting",
    },
]);

export const notifications = ref<NotificationData[]>([
    {
        id: 1,
        timestamp: new Date(),
        message: "BoxBot is starting",
    },
]);
