import random

from paho.mqtt import client as mqtt_client
from thermals.evo_thermal import EvoThermal


broker = 'localhost'
port = 1883
topic = "thermal/img"
topic_temp = "thermal/max_temp"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)

    return client


def publish_temps(evo, client):
    while True:
        img = evo.get_thermal_img()
        max_temp = evo.get_max_thermal()

        result = client.publish(topic, img)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            i = f"Send img to topic `{topic}`"
        else:
            print(f"Failed to send message to topic {topic}")

        result = client.publish(topic_temp, max_temp)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{max_temp}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")



def run():
    client = connect_mqtt()
    evo = EvoThermal()

    client.loop_start()
    # publish_max_temp(evo, temp_client)
    publish_temps(evo, client)


if __name__ == '__main__':
    run()
