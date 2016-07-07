#!/usr/bin/env python

import zmq
import rospy
import threading
from robotics_task_tree_eval.msg import *

def SubThread(sub, cb, event):
    count = 0
    timeout = False
    while event.is_set():
        try:
            [address, msg] = sub.recv_multipart()
            cb(msg)
            timeout = False
        except zmq.ZMQError as e:
            if e.errno == zmq.ETERM:
                break
            elif e.errno == zmq.EAGAIN:
                timeout = True
            else:
                raise
        count += 1

class NodePeerConnectionInterface:
    def __init__(self, node_name, peer_list, server_params, running_event):
        self.node_name = node_name
        self.peer_list = peer_list
        self.server_params = server_params
        self.context = zmq.Context()
        self.running_event = running_event

        # Setup ros subscription
        self.InitializeSubscriber(node_name)
        self.InitializePublisher(node_name)

    def InitializeSubscriber(self, name):
        # Setup peer listener
        self.peer_sub = rospy.Subscriber('%s%s'%(name, '_peer'), ControlMessage, self.ReceiveFromPeer, queue_size=5)

        # setup zeromq publisher
        self.pub = self.CreateZMQPub()

    def InitializePublisher(self, name):
        # setup peer publisher
        self.peer_pub = rospy.Publisher('%s%s'%(name, '_peer'), ControlMessage, queue_size=5)

        # setup zeromq subscriber
        self.sub = self.CreateZMQSub('%s%s'%(name, '_peer'), self.SendToPeer)

    def ReceiveFromPeer(self, msg):
        # Publish to server
        print 'Received from peer'
        # self.pub.send_multipart(self.node_name, msg)

    def SendToPeer(self, msg):
        print 'send to peer'
        # self.peer_pub.publish(msg)
        

    def CreateZMQPub(self):
        publisher = self.context.socket(zmq.PUB)
        publisher.bind("tcp://*:%s"%(self.server_params.pub_port))
        return publisher

    def CreateZMQSub(self, topic, cb):
        subscriber = self.context.socket(zmq.SUB)
        subscriber.connect("tcp://%s:%s"%(self.server_params.address, self.server_params.sub_port))
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        subscriber.setsockopt(zmq.RCVTIMEO, 1000)

        # Create thread
        thread = threading.Thread(target=SubThread, args=[subscriber, cb, self.running_event])
        thread.start()

        return subscriber

def main():
    # Initialize Node
    rospy.init_node('interface')
    class ServerParam:
        def __init__(self):
            pass
    server = ServerParam
    server.sub_port = '5565'
    server.pub_port = '5566'
    server.address = 'localhost'
    running_event = threading.Event()
    running_event.set()
    print 'Creating Nodes'

    node_list = rospy.get_param('~NodeList', None)

    for node in node_list:
        print node
    interface = NodePeerConnectionInterface('test', ['test2', 'test3'], server, running_event)
    print 'Spinning'
    rospy.spin()
    print 'shutting Down node'
    if rospy.is_shutdown():
        print 'Clearing event'
        running_event.clear()

if __name__ == '__main__':
    main()
