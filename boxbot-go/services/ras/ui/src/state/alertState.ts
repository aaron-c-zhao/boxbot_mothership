import { ref } from "@vue/composition-api";

export const ERROR_MESSAGE = "An error has occurred, please try again later";

export type AlertState = {
    enabled: boolean;
    text: string;
    type: "alert" | "warning" | "info" | "error" | "success";
};

export const alertState = ref<AlertState>({
    enabled: false,
    text: "",
    type: "info",
});
