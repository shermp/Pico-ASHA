import QtCore
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import PicoASHAGui

Window {
    id: root
    width: 800
    height: 600
    visible: true
    title: qsTr("Pico-ASHA")

    Dialog {
        id: errDialog
        anchors.centerIn: parent
        title: "Error"
        standardButtons: Dialog.Ok
        visible: false
        modal: true
        onAccepted: PicoAshaComm.errMsg = ""
        Label {
            text: PicoAshaComm.errMsg
        }
    }

    Connections {
        target: PicoAshaComm
        function onErrMsgChanged() {
            if (PicoAshaComm.errMsg !== "") {
                errDialog.visible = true
            }
        }
    }

    Page {
        id: ashaPage
        width: parent.width
        height: parent.height
        padding: 5
        header: Label {
            text: PicoAshaComm.serialConnected ? "Connected - Firmware %1".arg(PicoAshaComm.paFirmwareVers) : "Not Connected"
            font.pointSize: 10
            color: PicoAshaComm.serialConnected ? "white" : "black"
            padding: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            background: Rectangle{
                color: PicoAshaComm.serialConnected ? "green" : "grey"
                width: parent.width
            }
        }
        footer: ScrollView {
            id: footer
            width: parent.width
            height: 100
            padding: 10
            background: Rectangle {
                border.color: "black"
                color: "transparent"
            }

            ListView {
                id: logView
                model: PicoAshaComm.logModelList()
                delegate: Label {
                    text: `<pre>${model.display}</pre>`
                    textFormat: Text.StyledText
                }
                onCountChanged: logView.currentIndex = count - 1
            }
        }

        ColumnLayout {
            anchors.fill: parent
            ListView {
                id: remoteGrid
                model: PicoAshaComm.remoteModelList()
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignHCenter
                orientation: Qt.Horizontal
                delegate: GroupBox {
                    id: remotegroup
                    title: {
                        var side = model.remote.side
                        return side === RemoteDevice.Left ? "Left"
                                                          : side === RemoteDevice.Right ? "Right"
                                                                                        : "Unknown"
                    }
                    GridLayout {
                        columns: 2
                        Label { text: "Conn ID"; font.bold: true }
                        Label { text: `${model.remote.connID}` }

                        Label { text: "HCI Handle"; font.bold: true }
                        Label { text: `0x${model.remote.hciConnHandle.toString(16)}` }

                        Label { text: "Manufacturer"; font.bold: true }
                        Label { text: model.remote.mfgName }

                        Label { text: "Model"; font.bold: true }
                        Label { text: model.remote.modelName }

                        Label { text: "Firmware Version"; font.bold: true }
                        Label { text: model.remote.fwVersion }

                        Label { text: "Device Name"; font.bold: true }
                        Label { text: model.remote.deviceName }

                        Label { text: "Streaming Audio"; font.bold: true }
                        Label { text: model.remote.audioStreaming ? "Yes" : "No" }

                        Label { text: "Volume"; font.bold: true }
                        Label { text: model.remote.currVolume }
                    }
                }
            }
            RowLayout {
                Layout.fillWidth: true
                Layout.minimumHeight: 30
                Layout.maximumHeight: 30
                TextField {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 8
                    id: hciPath
                    focus: true
                    readOnly: true
                    onTextChanged: PicoAshaComm.hciLoggingPath = hciPath.text
                }
                Button {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    text: qsTr("HCI Log Path")
                    enabled: !PicoAshaComm.hciLoggingEnabled
                    onClicked: hciFileDialog.open()
                }
                Button {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    text: PicoAshaComm.hciLoggingEnabled ? qsTr("Stop HCI Log") : qsTr("Start HCI Log")
                    onClicked: PicoAshaComm.hciLoggingEnabled = !PicoAshaComm.hciLoggingEnabled
                }
                FileDialog {
                    id: hciFileDialog
                    currentFolder: StandardPaths.standardLocations(StandardPaths.DocumentsLocation)[0]
                    fileMode: FileDialog.SaveFile
                    nameFilters: ["BT Snoop Files (*.log)"]
                    onAccepted: hciPath.text = selectedFile
                }
            }
        }
    }
}
