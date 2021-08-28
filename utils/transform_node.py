#!/usr/bin/env python

import rospy
import tf
import math as m


"""

Export module.

This class will subscribe TF message and save tf matrix into self.trans, self.rot

"""


class TransformNode(object):
    def __init__(self, parent, child):
        super(TransformNode, self).__init__()

        self.parent = parent
        self.child = child

        self.listener = tf.TransformListener()

        self.trans = [0.0, 0.0, 0.0]
        self.rot = [0.0, 0.0, 0.0, 1.0]

    def listenTF(self):
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.parent, self.child, rospy.Time(0))
            self.trans = trans
            self.rot = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        except Exception as ex:
            print(ex)
