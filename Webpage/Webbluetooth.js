var connected = false;
        var services_discovered = false;
        var selected_device;
        var connected_server;
        // presence of services and characteristics
        var has_environmental_service = false;
        var has_temperature_data = false;
        TEMPERATURE_DATA = '00002a6e-0000-1000-8000-00805f9b34fb';
        //cached characteristics
        var temperature_data;
        var notifications_enabled = false;

        function setConnectedStatus(status) {
            connected = status;
            document.getElementById('status_connected').innerHTML = status;
            if (status == true) {
                document.getElementById('btn_scan').innerHTML = "Disconnect";
            } else {
                document.getElementById('btn_scan').innerHTML = "Discover Devices";
            }
        }
        function setDiscoveryStatus(status) {
            services_discovered = status;
            document.getElementById('status_discovered').innerHTML = status;
        }

        function discoverDevicesOrDisconnect() {
            console.log("discoverDevicesOrDisconnect");
            if (!connected) {
                discoverDevices();
            } else {
                selected_device.gatt.disconnect();
                resetUI();
                // TODO disconnect from the current device
            }
        }
        ENVIRONMENTAL_SERVICE = 0x181a;
        function discoverDevices() {
            console.log("discoverDevices");
            var options = {
                //acceptAllDevices: true // can change the properties to filter by name and services UUID
                filters: [{ namePrefix: 'RGB'}],
                optionalServices: [ENVIRONMENTAL_SERVICE]
            }
            navigator.bluetooth.requestDevice(options)
                .then(device => {
                    console.log('> Name: ' + device.name);
                    console.log('> Id: ' + device.id);
                    console.log('> Connected: ' + device.gatt.connected);
                    selected_device = device;
                    console.log(selected_device);
                    connect();
                })
                .catch(error => {
                    alert('ERROR: ' + error);
                    console.log('ERROR: ' + error);
                });
        }

        function readTemperature() {
            console.log("read TEMPERATURE");
             // state validation
            if (!connected) {
            alert("Error: Discover and connect to a device before using this function");
            return;
            }
            if (!services_discovered) {
            alert("Error: Service discovery has not yet completed");
            return;
            }
            if (!has_temperature_data) {
                console.log("Error: Temperature Data characteristic not found!");
                return;
            }
            temperature_data.readValue().then(value => {
                console.log('Temperature Data: ' + value.getUint8(0));
                document.getElementById('Temperature').innerHTML = value.getUint8(0);
            }).catch(error => {
                console.log('Error reading Temperature Data: ' + error);
                alert('Error' + error);
                return;
            });
        }


        function toggleTemperatureNotifications() {
            console.log("toggleTemperatureNotifications");
            //setNotificationsStatus(true);
            if (!connected) {
                alert("Error: Discover and connect to a device before using this function");
                return;
            }
            if (!services_discovered) {
                alert("Error: Service discovery has not yet completed");
                return;
            }
            if (!notifications_enabled) {
                const clientconfigdescriptor = new ArrayBuffer([1, 1]);
                console.log("Starting Temperature Notifications");
                //temperature_data.startNotifications();
                
                temperature_data.getDescriptors().then(descriptors => {
                    let enableNotifications = new ArrayBuffer([1, 0]);
                    let queue = Promise.resolve();
                        descriptors.forEach(descriptor => {
                        queue = queue.then(_ => descriptor.readValue()).then(value => {
                            let notificationsBit = value.getUint8(0) & 0b01;
                            console.log('  > Notifications: ' + (notificationsBit ? 'ON' : 'OFF'));
                            console.log('Descriptor Value: ' + descriptor.value);
                            temperature_data.addEventListener('characteristicvaluechanged',onTempData);
                            temperature_data.startNotifications();
                        });
                        //descriptor.writeValue(enableNotifications);
                    });
                });
            } else {
                temperature_data.stopNotifications()
                .then(_ => {
                    console.log('temperature notifications stopped');
                    setNotificationsStatus(false);
                    temperature_data.removeEventListener('characteristicvaluechanged',onTempData);
                })
                .catch(error => {
                    console.log('Could not stop Temp_data notifications: ' + error);
                });
            }
        }

        function connect() {
            if (connected == false) {
                console.log("connecting");
                selected_device.gatt.connect().then(
                    function (server) {
                        console.log("Connected to " + server.device.id);
                        console.log("connected=" + server.connected);
                        setConnectedStatus(true);
                        connected_server = server;
                        selected_device.addEventListener('gattserverdisconnected', onDisconnected);
                        discoverSvcsAndChars();
                        console.log("Services Discovered!!");
                        //toggleTemperatureNotifications();
                        
                    },
                    function (error) {
                        console.log("ERROR: could not connect - " + error);
                        alert("ERROR: could not connect - " + error);
                        setConnectedStatus(false);
                    });
            }
        }


        function onDisconnected() {
            console.log("onDisconnected");
            resetUI();
        }



        function discoverSvcsAndChars() {
            console.log("discoverSvcsAndChars server=" +
                connected_server);
            connected_server.getPrimaryServices()
                .then(services => {
                    has_environmental_service = false;
                    has_led_service = false;
                    has_device_information_service = false;
                    services_discovered = 0;
                    service_count = services.length;
                    console.log("Got " + service_count + " services");
                    services.forEach(service => {
                        if (service.uuid == ENVIRONMENTAL_SERVICE) {
                            has_environmental_service = true;
                        }
    
                        console.log('Getting Characteristics for service ' + service.uuid);
                        service.getCharacteristics().then(characteristics => {
                            console.log('> Service: ' + service.uuid);
                            services_discovered++;
                            characteristics_discovered = 0;
                            characteristic_count =
                                characteristics.length;
                            characteristics.forEach(characteristic => {
                                characteristics_discovered++;
                                console.log('>> Characteristic: ' + characteristic.uuid);
                            if (characteristic.uuid == TEMPERATURE_DATA) {
                                    temperature_data = characteristic;
                                    has_temperature_data = true;
                                }
                                if (services_discovered == service_count && characteristics_discovered == characteristic_count) {
                                    console.log("FINISHED DISCOVERY");
                                    setDiscoveryStatus(true);
                                }
                            });
                        });
                    });
                });
        }
        function setConnectedStatus(status) {
            connected = status;
            //document.getElementById('status_connected').innerHTML =
                status;
            if (status == true) {
                document.getElementById('btn_scan').innerHTML =
                    "Disconnect";
            } else {
                document.getElementById('btn_scan').innerHTML =
                    "Discover Devices";
            }
        }
        function setDiscoveryStatus(status) {
            services_discovered = status;
            //document.getElementById('status_discovered').innerHTML =
                //status;
        }
        function resetUI() {
            setConnectedStatus(false);
            setDiscoveryStatus(false);
            setNotificationsStatus(false);
            //document.getElementById('Temperature').innerHTML = "";
            document.getElementById('temperature_data').innerHTML = "";
        }
        function setNotificationsStatus(status) {
            /*Customise For Temperature DATA*/
            notifications_enabled = status;
            
        //document.getElementById('status_notifications').innerHTML = status;
       
        }
        // neet to change this for temperature data
        function onTempData(event) {
            console.log("onTempData");
            buffer = event.target.value.buffer;
            dataview = new DataView(buffer);
            Temp = dataview.getUint32(0, true);
            console.log("Temperature Data: " + Temp);
            document.getElementById("temperature_data").innerHTML = Temp+"&#8451;";
        }