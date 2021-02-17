<template>
    <span>
        <v-expansion-panels v-model="filterOpen">
            <v-expansion-panel>
                <v-expansion-panel-header>Filters</v-expansion-panel-header>
                <v-expansion-panel-content>
                    <v-container>
                        <v-row>
                            <v-datetime-picker
                                v-model="dateTimeAfter"
                                label="Only show after"
                                :time-picker-props="{ format: '24h' }"
                            >
                                <template slot="dateIcon">
                                    <v-icon>mdi-calendar</v-icon>
                                </template>
                                <template slot="timeIcon">
                                    <v-icon>mdi-clock</v-icon>
                                </template></v-datetime-picker
                            >
                            <v-spacer />
                            <v-datetime-picker
                                v-model="dateTimeBefore"
                                label="Only show before"
                                :time-picker-props="{ format: '24h' }"
                            >
                                <template slot="dateIcon">
                                    <v-icon>mdi-calendar</v-icon>
                                </template>
                                <template slot="timeIcon">
                                    <v-icon>mdi-clock</v-icon>
                                </template></v-datetime-picker
                            >
                        </v-row>
                        <v-row>
                            <v-select v-model="levelFilter" :items="logLevels" label="Filter on log level" multiple />
                        </v-row>
                    </v-container>
                    <v-btn color="primary" class="text-capitalize px-2 mt-5" style="color: white" @click="applyFilters">
                        Apply filters
                    </v-btn>
                </v-expansion-panel-content></v-expansion-panel
            ></v-expansion-panels
        >

        <v-data-table
            :options.sync="options"
            :headers="headers"
            :items="logData"
            item-key="id"
            :item-class="style"
            :loading="loading"
            :server-items-length="42"
            :footer-props="{ itemsPerPageOptions: [10, 50, 100, -1] }"
            @update:options="loadLogs"
        >
            <template v-slot:item.timestamp="{ item }">
                <span>{{ new Date(item.timestamp).toLocaleString() }}</span>
            </template>
        </v-data-table>

        <v-btn color="primary" class="text-capitalize mx-2 mt-1" style="color: white" @click="loadLogs">
            Refresh
        </v-btn>
        <v-btn color="error" class="text-capitalize mx-2 mt-1" style="color: white" @click="deleteLogs"> Delete </v-btn>
    </span>
</template>

<script lang="ts">
import { defineComponent, onMounted, ref } from "@vue/composition-api";
import { DataOptions } from "vuetify";
import { alertState } from "@/state/alertState";
import { LogEntry, LogLevel } from "@/api/logs";

export default defineComponent({
    setup() {
        const filterOpen = ref([]);
        const levelFilter = ref<LogLevel[]>(Object.keys(LogLevel));
        const dateTimeAfter = ref<Date | null>(null);
        const dateTimeBefore = ref<Date | null>(null);
        const logData = ref<LogEntry[]>([
            {
                id: 1,
                timestamp: new Date(),
                level: LogLevel.INFO,
                message: "BoxBot is starting",
            },
            {
                id: 2,
                timestamp: new Date(),
                level: LogLevel.WARNING,
                message: "Pallet is nearly empty",
            },
            {
                id: 3,
                timestamp: new Date(),
                level: LogLevel.ERROR,
                message: "Motor C is stalling",
            },
        ]);
        const loading = ref(true);
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

        function applyFilters() {
            filterOpen.value = [];
            loadLogs();
        }

        function loadLogs() {
            loading.value = true;

            setTimeout(() => {
                loading.value = false;
            }, 500);
        }

        function deleteLogs() {
            alertState.value = {
                enabled: true,
                type: "info",
                text: "Deleted logs for filters",
            };

            logData.value = [];
        }

        onMounted(() => {
            loadLogs();
        });

        return {
            filterOpen,
            levelFilter,
            dateTimeAfter,
            dateTimeBefore,
            applyFilters,
            loadLogs,
            deleteLogs,
            logData,
            loading,
            options,

            logLevels: Object.keys(LogLevel),
            headers: [
                { text: "Timestamp", value: "timestamp", sortable: false, width: "20%" },
                { text: "Level", value: "level", sortable: false, width: "10%" },
                { text: "Message", value: "message", sortable: false, width: "70%" },
            ],
            style: function (e: LogEntry) {
                switch (e.level) {
                    case LogLevel.ERROR:
                        return "log-error";
                    case LogLevel.WARNING:
                        return "log-warn";
                    default:
                        return "";
                }
            },
        };
    },
});
</script>

<style lang="scss">
.log-error {
    background-color: rgb(245, 198, 203);
}

.log-warn {
    background-color: rgb(255, 238, 186);
}
</style>
