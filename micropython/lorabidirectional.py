import time
import configurationlora
from connectwlan import do_connect
import urequests as requests

id1 = ''
id2 = ''
id3 = ''

ssid = ""
pswd = ""

humidity = []
temperature = []

wlan = do_connect(ssid, pswd)
s = wlan.config("mac")
wifimac = ('MAC:%02x-%02x-%02x-%02x-%02x-%02x ').upper() % (s[0], s[1], s[2], s[3], s[4], s[5])

print(wifimac)

if not wlan.isconnected():
    print('connecting to network...' + ssid)
    wlan.connect(ssid, pswd)

msgCount = 0  # count of outgoing messages
INTERVAL = 2000  # interval between sends
INTERVAL_BASE = 2000  # interval between sends base


def duplexCallback(lora):
    print("Lora bidirectional with callback")
    lora.onReceive(on_receive)  # register the reception of the message

    do_loop(lora)


def do_loop(lora):
    global msgCount
    lastSendTime = 0
    interval = 0

    while True:
        now = configurationlora.millisecond()
        if now < lastSendTime: lastSendTime = now
        if (now - lastSendTime > interval):
            lastSendTime = now  # timestamp the message
            interval = (lastSendTime % INTERVAL) + INTERVAL_BASE  # 2-3 seconds

            message = "{} {}".format('Hello LoRa!', '#' + str(msgCount) + str(configurationlora.NODE_NAME))
            sendMessage(lora, message)  # send message
            msgCount += 1
            lora.receive()  # go into receive mode


def sendMessage(lora, outgoing):
    lora.println(outgoing)
    print("Sending message:\n{}\n".format(outgoing))


def on_receive(lora, payload):
    lora.blink_led()
    rssi = lora.packetRssi()
    try:
        length = len(payload) - 1
        myStr = str((payload[4:length]), 'utf-8')
        length1 = myStr.find('H:')
        myNum1 = myStr[(length1 + 2):(length1 + 7)]
        print('%s\n' % (myStr))

        length2 = myStr.find('T:')
        myNum2 = myStr[(length2 + 2):(length2 + 7)]

        print("Number one is{} ".format(myNum1))
        print("")
        print("Number two is{}".format(myNum2))
        print("*** Received message ***\n{} ".format(payload))

        if configurationlora.IS_LORA_OLED: lora.show_packet(("{}".format(payload[3:length])), rssi)

        b = False
        if wlan.isconnected():
            global msgCount
            print('Sending to network...')

            node = int(str(payload[3:8], 'utf-8'))
            print('%s NODE NAME\n' % (node))

            # replace "" with server address

            if node == 10514:
                URL = "" + id1 + "&field1=" + myNum1 + "&field2=" + myNum2
                res = requests.get(URL)
                b = True
            # print(res.text)

            if node == 10461:
                URL = "" + id2 + "&field1=" + myNum1 + "&field2=" + myNum2
                res = requests.get(URL)
                b = True
                # print(res.text)

            if node == 10463:
                URL = "" + id3 + "&field1=" + myNum1 + "&field2=" + myNum2
                res = requests.get(URL)
                b = True
                # print(res.text)

        if not b:
            on_receive(lora, payload)

    except Exception as e:

        print(e)
    print("with RSSI {}\n".format(rssi))
