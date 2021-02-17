const localEnv = process.env.NODE_ENV === "development";
const localPort = 8081;

module.exports = {
    productionSourceMap: !localEnv,
    devServer: {
        port: localPort,
    },
    pwa: {
        name: "BoxBot Maintenance",
        iconPaths: {
            favicon32: null,
            favicon16: "img/icons/favicon-16x16.png",
            appleTouchIcon: null,
            maskIcon: null,
            msTileImage: null,
        },
    },
    chainWebpack: (config) => {
        config.plugin("define").tap((definitions) => {
            definitions[0].DEV_ENABLED = JSON.stringify(localEnv);

            return definitions;
        });

        config.plugin("html").tap((args) => {
            args[0].title = "BoxBot Maintenance";
            return args;
        });
    },
};
