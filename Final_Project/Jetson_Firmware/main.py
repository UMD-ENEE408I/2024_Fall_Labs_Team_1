import threading
import time
import random
import json
import client_server, audio_processing, image_processing
import logging
logging.basicConfig(format='[%(threadName)-16.16s] %(levelname)s: %(message)s')
logging.getLogger().setLevel(logging.INFO)

def main():

    # "Buzz 4 Times"
    encrypted_buzz4 = [
        'AA858E01C63EB9EB73FF61E9562CB9A05D8F1EB19497FAA5578DC630000110A6FDF188F17072529F892B02DC8DABE21EAF1F36226CBF2A58D33EA0F1A56B758F',
        '291850F82320C50E27DA86D0FDAA2EE08C8B3B708D0637E349525A43BC340B6A577D07AB38ECB98F99542EE42691E95D6FCE2703CD0E115552239DCE4D8BF9E1',
        'DB6455A0582DC3E199C48FF13E62D3776A0D0F5B4EA188C0C0C3FEDFECDF3443416D357B7B3C0270283FF4E7F16EA8F7B219CD9B8DC03A038A3CDA2AA80D6A84',
        'FFE81F161654AEEF77A16E37D7E26CFCDBEE8DDB4C58C6BD1205FFAD8FC13EA155047D0FC3C0EE4D929D8E2B8D392CF4A8A40F350BF79E8749609EFF8933EE7A',
        '45677C78D54054CE69DEA602E8108DFD9B5F1F389DF6658A5781C7731CC5CCF1071A0A8314AEBDA47EE1DE641411A9D6F91EC200FF78F84787C62576E57D8BCF'
    ]

    # "Buzz 5 Times"
    encrypted_buzz5 = [
        '076159BB81CE466C12DADE5B4CB0386FF10054E6EFFFFC5ECE5E679A50AC64960ADC5A75110BC49EDE042B8422AB207EAF1F36226CBF2A58D33EA0F1A56B758F', # enee408ikeynumb1
        '1FD1235816F4463862EC2C9E85581C7C6A67D650DD55CAAAE6CAE813F400240284AB8817BB63098EFAFBC278D067F9B86FCE2703CD0E115552239DCE4D8BF9E1', # keynumb2enee408i
        '19624CFA8B103C771C632A38F3FC05D0A3C2F7812481545A6C0DC752633CC824F8C095A8E9E5BEE10086149E54BAD787B219CD9B8DC03A038A3CDA2AA80D6A84', # keythreeenee408i
        '60AE6E9CB1BA9B97B38420064931C08F9CFAC097C9427EFBF9794AC86D711FB747B8E9CA587D300277CD38AE1A3AB26DA8A40F350BF79E8749609EFF8933EE7A', # capstone8keyfour
        'B6B1C49F6B0AFF936E902088E10347C62C91BFDC826A18537F200654F3FF66C787EBA5A12D625C701E890D9628ADDE6BF91EC200FF78F84787C62576E57D8BCF'  # thefifthkeyfor8i
    ]

    encrypted_messages = [encrypted_buzz4, encrypted_buzz5]

    buzz_count = random.randrange(0, len(encrypted_messages))
    plaintext_msg = f'Buzz {buzz_count+4} Times'
    encrypted_choice = random.randrange(0, 5)

    missing_block = encrypted_messages[buzz_count][encrypted_choice][0:16]

    logging.info(f'The Robot should buzz {buzz_count + 4} times!')
    logging.info(f'Encrypted message: {encrypted_messages[buzz_count][encrypted_choice]} encrypted with key {encrypted_choice+1}')
    logging.info(f'The Missing Block is: {missing_block}')

    msg_to_D = {
        'op': 'init_encrypt',
        'name': 'BotD',
        'encrypted': encrypted_messages[buzz_count][encrypted_choice][16:]
    }

    threading.Thread(name='ESP Server Start', target=client_server.start_server(plaintext_msg, msg_to_D), daemon=True).start()
    threading.Thread(name='Audio Processing Start', target=audio_processing.start_handler_thread, daemon=True).start()
    threading.Thread(name='Image Processing Start', target=image_processing.start_handler_thread, daemon=True).start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break
    logging.info("Exiting")


if __name__ == '__main__':
    main()