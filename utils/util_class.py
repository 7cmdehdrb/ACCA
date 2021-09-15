#!/usr/bin/env python


class Obstacle(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Map(object):
    def __init__(self):
        self.obstacles = []
