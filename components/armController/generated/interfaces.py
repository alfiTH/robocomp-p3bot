import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()

Ice.loadSlice("-I ./generated/ --all ./generated/KinovaArm.ice")
import RoboCompKinovaArm
Ice.loadSlice("-I ./generated/ --all ./generated/KinovaArmPub.ice")
import RoboCompKinovaArmPub
Ice.loadSlice("-I ./generated/ --all ./generated/VRControllerPub.ice")
import RoboCompVRControllerPub

class TJointSeq(list):
    def __init__(self, iterable=list()):
        super(TJointSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).insert(index, item)
setattr(RoboCompKinovaArm, "TJointSeq", TJointSeq)

class Speeds(list):
    def __init__(self, iterable=list()):
        super(Speeds, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(Speeds, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(Speeds, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(Speeds, self).insert(index, item)
setattr(RoboCompKinovaArm, "Speeds", Speeds)

class Angles(list):
    def __init__(self, iterable=list()):
        super(Angles, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(Angles, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(Angles, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(Angles, self).insert(index, item)
setattr(RoboCompKinovaArm, "Angles", Angles)


import vrcontrollerpubI

class Publishes:
    def __init__(self, ice_connector:Ice.CommunicatorI, topic_manager, parameters):
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager

        self.kinovaarmpub = self.create_topic("KinovaArmPub", "KinovaArmPub", parameters["Proxies"]["KinovaArmPubPrefix"], RoboCompKinovaArmPub.KinovaArmPubPrx)
        self.kinovaarmpub1 = self.create_topic("KinovaArmPub1", "KinovaArmPub", parameters["Proxies"]["KinovaArmPubPrefix1"], RoboCompKinovaArmPub.KinovaArmPubPrx)


    def create_topic(self, property_name, topic_name, prefix, ice_proxy):
        topic = False
        topic_full_name = f"{prefix}/{topic_name}" if prefix else topic_name

        while not topic:
            try:
                topic = self.topic_manager.retrieve(topic_full_name)
            except IceStorm.NoSuchTopic:
                try:
                    console.log(f"{Text('WARNING', style='yellow')} {topic_full_name} topic did not create. {Text('Creating...', style='green')}")
                    topic = self.topic_manager.create(topic_full_name)
                except:
                    console.log(f"{Text('WARNING', style='yellow')}publishing the {topic_full_name} topic. It is possible that other component have created")

        pub = topic.getPublisher().ice_oneway()
        proxy = ice_proxy.uncheckedCast(pub)
        self.mprx[property_name] = proxy
        return proxy

    def get_proxies_map(self):
        return self.mprx


class Requires:
    def __init__(self, ice_connector:Ice.CommunicatorI, parameters):
        self.ice_connector = ice_connector
        self.mprx={}

        self.KinovaArm = self.create_proxy("KinovaArm", RoboCompKinovaArm.KinovaArmPrx, parameters["Proxies"]["KinovaArm"])
        self.KinovaArm1 = self.create_proxy("KinovaArm1", RoboCompKinovaArm.KinovaArmPrx, parameters["Proxies"]["KinovaArm1"])

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy, proxy_string):
        try:
            base_prx = self.ice_connector.stringToProxy(proxy_string)
            proxy = ice_proxy.uncheckedCast(base_prx)
            self.mprx[property_name] = proxy
            return True, proxy
        
        except Ice.Exception as e:
            console.print_exception(e)
            console.log(f'Cannot get {property_name} property.')
            self.status = -1
            return False, None


class Subscribes:
    def __init__(self, ice_connector:Ice.CommunicatorI, topic_manager, default_handler, parameters):
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

        self.VRControllerPub = self.create_adapter("VRControllerPub", parameters["Endpoints"]["VRControllerPubPrefix"], 
                                                vrcontrollerpubI.VRControllerPubI(default_handler, ""), parameters["Endpoints"]["VRControllerPubTopic"])

    def create_adapter(self, topic_name, prefix, interface_handler, endpoint_string):
        topic_full_name = f"{prefix}/{topic_name}" if prefix else topic_name

        adapter = self.ice_connector.createObjectAdapterWithEndpoints(topic_full_name, endpoint_string)
        handler = interface_handler
        proxy = adapter.addWithUUID(handler).ice_oneway()
        subscribe_done = False
        while not subscribe_done:
            try:
                topic = self.topic_manager.retrieve(topic_full_name)
                subscribe_done = True
            except Ice.Exception as e:
                try:
                    console.log(f"{Text('WARNING', style='yellow')} {topic_full_name} topic did not create. {Text('Creating...', style='green')}")
                    topic = self.topic_manager.create(topic_full_name)
                    subscribe_done = True
                except:
                    print(f"{Text('WARNING', style='yellow')}publishing the {topic_full_name} topic. It is possible that other component have created")
        qos = {}
        topic.subscribeAndGetPublisher(qos, proxy)
        adapter.activate()
        return adapter


class Implements:
    def __init__(self, ice_connector:Ice.CommunicatorI, default_handler, parameters):
        self.ice_connector = ice_connector

    def create_adapter(self, property_name, interface_handler, endpoint_string):
        try:
            adapter = self.ice_connector.createObjectAdapterWithEndpoints(property_name, endpoint_string)
            adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
            adapter.activate()
            console.log(f"{property_name} adapter created in port {endpoint_string}")
        except:
            console.log(f"{Text('ERROR', style='red')} creating or activating adapter for{property_name}")
            self.status = -1


class InterfaceManager:
    def __init__(self, configData):

        init_data = Ice.InitializationData()
        init_data.properties = Ice.createProperties()
        init_data.properties.setProperty("Ice.Warn.Connections", configData["Ice"]["Warn"]["Connections"])
        init_data.properties.setProperty("Ice.Trace.Network", configData["Ice"]["Trace"]["Network"])
        init_data.properties.setProperty("Ice.Trace.Protocol", configData["Ice"]["Trace"]["Protocol"])
        init_data.properties.setProperty("Ice.MessageSizeMax", configData["Ice"]["MessageSizeMax"])
        self.ice_connector = Ice.initialize(init_data)

        self.status = 0

        needs_rcnode = True
        self.topic_manager = self.init_topic_manager(configData) if needs_rcnode else None

        self.requires = Requires(self.ice_connector, configData)
        self.publishes = Publishes(self.ice_connector, self.topic_manager, configData)
        self.implements = None
        self.subscribes = None

    def init_topic_manager(self, configData):
        obj = self.ice_connector.stringToProxy(configData["Proxies"]["TopicManager"])
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            self.status = -1
            exit(-1)

    def set_default_hanlder(self, handler, configData):
        self.implements = Implements(self.ice_connector, handler, configData)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler, configData)

    def get_proxies_map(self):
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




