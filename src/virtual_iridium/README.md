#What is this project?

This is a Iridium Modem (9602/9603) emulator to make it possible to test your own software again a "Virtual" Iridium device and it connection.

This application is written in Python language uses pyserial to implement a serial communications interface. The emulator matches the behavior of the Iridium 9602 modem which is available from NAL Research and Rock7Mobile (as Rockblock). The emulator will respond to at commands to write data, execute short-burst data (SBD) sessions, and most of the other functions supported by the 9602 serial interface.

#What's the use?

If you want to develop an application on a PC or an embedded device it will to talk to an Iridium modem, you can use this for initial prototyping and testing. This can potentially save quite some cost on Iridium service charges. Also you can already create an application without already buying the real Iridium Modem (9602/9603) hardware.

# How do I get this software?

In your Unix shell of choice:
```
 $ git clone https://github.com/jmalsbury/virtual_iridium
 $ cd virtual_iridium/python
 FOR EMAIL MODE:
 $ python Iridium9602.py -d /dev/ttyUSB0 -u youraccount@gmail.com -p your_password -i imap.gmail.com -o smtp.gmail.com -r your_iridium_test_account@gmail.com -m EMAIL

 FOR HTTP_POST MODE:
 $ python2 Iridium9602.py --webhook_server_endpoint <endpoint> --http_server_port <port #> -d <serial_device> -m HTTP_POST
   
   <webhook_server_endpoint> where to post webhooks to. Includes port #. Ex: "127.0.0.1:6665".

   <http_server_port> the port the iridium http server will be running on.

   <serial_device> one of the socat pairs created. READ the bbb_rockblock_listener README for more info. 

   (ex. python2 Iridium9602.py --webhook_server_endpoint localhost:8000 --http_server_port 8080 -d /dev/pts/5 -m HTTP_POST)  
```
The specified serial device, in the example above: ttyUSB0 , should connect to the external device that you are developing your Iridium communications app on. You can also use a virtual serial port (like a pair of SOCAT TTYs), to connect to another application on the same PC.

# Where can I find more documentation?

This is all I am going to write for now. If I see more people are interested in using this emulator, I may put more effort into this documentation. If you have any question, don't hesitate to e-mail me: jmalsbury [dot] personal [at] gmail [dot] com.
Other Resources

Iridium 9602 - Developers Manual{TODO}Link to repofile

Thanks to the original autor J.Malsbury. Maybe his repo is more up to date or has mre features.
[![githalytics.com alpha](https://cruel-carlota.pagodabox.com/bdb824945e9b3e4a959feb550731a1e0 "githalytics.com")](http://githalytics.com/jmalsbury/virtual_iridium)
