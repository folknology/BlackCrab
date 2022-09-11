#Debugnotes
- [ ] Probe RS VSCode https://github.com/probe-rs/vscode/releases/tag/v0.4.0
```bash
code --install-extension probe-rs-debugger-0.4.0.vsix
```
- [ ] Setup https://probe.rs/docs/tools/vscode/#installation

Launch configuration

```json
{
    "version": "0.2.0",
    "configurations": [
        
        {
            "preLaunchTask": "${defaultBuildTask}", //Configure a default build task for 'cargo build'
            "type": "probe-rs-debug",
            "request": "launch",
            "name": "probe_rs debug",
            "cwd": "${workspaceFolder}",
            "connectUnderReset": true,
            "speed": 24000, //!MODIFY (or remove)
            //"probe": "0483:374f:<Serial>", //!MODIFY (or remove)
            "runtimeExecutable": "probe-rs-debugger",
            "runtimeArgs": [
                "debug"
            ],
            "chip": "STM32F730V8Tx", //!MODIFY
            "flashingConfig": {
                "flashingEnabled": true,
                "resetAfterFlashing": true,
                "haltAfterReset": true,
            },
            "coreConfigs": [
                {
                    "coreIndex": 0,
                    "programBinary": "${workspaceFolder}/target/thumbv7em-none-eabihf/debug/black_crab", //!MODIFY
                    "svdFile": "../cmsis-svd/data/STMicro/STM32F7x.svd", //!MODIFY
                }
            ],
            "consoleLogLevel": "Info", //Error, Warn, Info, Debug, Trace 
        }
    ]
}
```