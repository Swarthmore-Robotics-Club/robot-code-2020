/*   
HC05 - Bluetooth AT-Command mode  
modified on 10 Feb 2019 
by Saeed Hosseini 
https://electropeak.com/learn/ 
*/ 
#include <SoftwareSerial.h> 

/*

This message protocol sends information over bluetooth. Right now a message consists of two parts

1) The message header. This is meta-data that helps other applications read the messages we send 

    right now the header contains
    a) (1 byte long) message length. How long the message is, we can add other things as we see fit
    b) (1 byte long) message command. 0 if not a specific command just junk to print, we can have up to 255 other commands

        for example:
        
            if the message command reads 1 we can agree that we are sending wall sensor values to be read

            if the mssage command reads 2 we can agree that they are encoder sensor values that are being read. and so on...


2) arbitrary data. This is data sent 

*/

//class BluetoothTerminal {
//  public:
//    BluetoothTerminal(byte tx_pin, byte rx_pin) {
//      bluetooth_module = new SoftwareSerial(tx_pin, rx_pin);
//    }
//
//    bool available() {
//      return bluetooth_module->available();
//    }
//
//    // read from the bluetooth module, parse the headers, make sense of it
//    String read() {
//      return String(bluetooth_module->read());
//    }
//
//    // encapsulate a passed in string/buffer of data to be sent over bluetooth
//    byte write(String string, byte message_cmd) {
//      // create message buffer and add header to it
//      char message[200];
//      byte message_len = string.length() + sizeof(struct message_header);
////      populate_header(message,message_len, messsage_cmd);
////      memcpy(message + sizeof(struct message_header), (void*)string, string.length()); 
//      // now write the message out over bluetooth
//
////      bluetooth_module.write(message);
//    }
//
//  private:
//    SoftwareSerial*  bluetooth_module;
//
//    struct message_header {
//      byte msg_len, message_cmd;
//    };
//    
//    // given a buffer and size and command we populate the header with it
//    void populate_header(char* buff, byte msg_size, byte message_cmd) {
//      struct message_header* header = (struct message_header*) buff; 
//      buff->msg_len = msg_size;
//      buff->message_cmd = message_cmd;
//    }
//};

SoftwareSerial MyBlue(2, 3); // RX | TX 
void setup() { 
  Serial.begin(9600); 
  MyBlue.begin(9600); 
  Serial.println("Give me an H!");
}

bool subscribed = false;
String input = "";

void loop() { 
  Serial.println("Give me an H please!");
  while (MyBlue.available() <= 0) {delay(100);} 
  input = MyBlue.read();
  if (input == "104") {
    MyBlue.write("Zane please");
    Serial.println("WE GOT AN H");
  }
}
 
