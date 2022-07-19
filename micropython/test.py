import sx127x
import configurationlora
import lorabidirectional


def main():
    controller = configurationlora.Controller()

    lora = controller.add_transceiver(sx127x.SX127x(name='LoRa'),
                                      pin_id_ss=configurationlora.Controller.PIN_ID_FOR_LORA_SS,
                                      pin_id_RxDone=configurationlora.Controller.PIN_ID_FOR_LORA_DIO0)
    print('lora', lora)

    lorabidirectional.duplexCallback(lora)


if __name__ == '__main__':
    main()
