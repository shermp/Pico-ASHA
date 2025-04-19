import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import PicoASHAGui

Window {
    id: root
    width: 800
    height: 600
    visible: true
    title: qsTr("Pico-ASHA")

    Page {
        id: ashaPage
        width: parent.width
        height: parent.height
        padding: 5
        header: Label {
            text: PicoAshaComm.serialConnected ? "Connected - Firmware %1".arg(PicoAshaComm.paFirmwareVers) : "Not Connected"
            font.pointSize: 10
            padding: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            background: Rectangle{
                color: PicoAshaComm.serialConnected ? "green" : "grey"
                width: parent.width
            }
        }
        ColumnLayout {
            width: parent.width
            GridView {
                model: PicoAshaComm.remoteModelList()
                width: parent.width
                cellWidth: 300
                delegate: GroupBox {
                    title: {
                        var side = model.remote.side
                        return side === RemoteDevice.Left ? "Left"
                                                          : side === RemoteDevice.Right ? "Right"
                                                                                        : "Unknown"
                    }
                    GridLayout {
                        columns: 2
                        width: parent.width
                        Label { text: "Conn ID"; font.bold: true }
                        Label { text: `${model.remote.connID}` }
                        Label { text: "HCI Handle"; font.bold: true }
                        Label { text: `0x ${model.remote.hciConnHandle.toString(16)}` }
                        Label { text: "Device Name"; font.bold: true }
                        Label { text: model.remote.deviceName }
                        Label { text: "Volume"; font.bold: true }
                        Label { text: `${model.remote.currVolume}` }
                    }
                }
            }
        }
        footer: ScrollView {
            width: parent.width
            height: 100
            padding: 10

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
    }
}
