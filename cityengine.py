"""Copyright (c) 2015 Francesco Mastellone

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

"""
The road system is implemented as a graph of inter-referenced Way objects.
Ways keep track of which Vehicles are travelling along them, to inform other
Vehicles of who's in front of them and how far ahead they are, so that they may
keep a safe distance.

Intersections are a considerable approximation in that only one Way at a time is
allowed through. Still figuring out ways to prevent the collisions that would
result otherwise.

Cars have their origin in the back.

Each Way has a traffic_light value, that Intersections set.
...There's surely a better way to do that.

"""

from random import random, choice
from math import hypot

import sys
sys.path.append('/usr/local/lib/python2.7/dist-packages')
import paho.mqtt.client as mqtt

import threading
import socket
from threading import Timer

import requests
import logging

global traffic_light,flag
traffic_light="green"
flag=0

MAX_POS = 9999.

class Intersection:
    def __init__(self, ways=None):
        self.ways = ways if ways else []
        self.i = 0 # Active way
        self.t = 0.
        self.go = True
        self.go_time = 6.
        self.stop_time = 2. # Yellow traffic light

    def update(self, dt):
        self.t -= dt

        if self.t < 0.:
            if self.go:
                self.ways[self.i].traffic_light = 'yellow'
                self.t = self.stop_time
                self.go = False
            else:
                self.ways[self.i].traffic_light = 'red'
                self.i += 1
                self.i %= len(self.ways)
                self.ways[self.i].traffic_light = 'green'
                self.t = self.go_time
                self.go = True

class Way:
    def __init__(self, speed_limit=13.89/8.):
        self.traffic_light = 'green'
        self.speed_limit = speed_limit # m/s
        self.to = [] # Directed graph neighbors
        self.cars = []

    def next_car(self, car):                  #查询小车的位置
        """Picks the next [pos, car] from self and self.to"""
        self.cars.sort(key=lambda car: car.pos)
        i = self.cars.index(car)
        if i + 1 < len(self.cars):
            car2 = self.cars[i + 1]
            return car2.pos, car2

        elif self.to and self.to[0].cars: # TODO foresee where car goes
            #print(self.to[0].cars)
            car2 = self.to[0].cars[0]
            return car2.pos + self.length, car2
        else:
            return None, None

    def next_obstacle_position(self, car):        #查询障碍物位置
        """Returns distance from obstacle obstacle.pos or None if no obstacles
        are in sight"""
        obpos = MAX_POS # Obstacle position
        global traffic_light,flag
        if self.to :
            flag=0
        else:
            if self.cars[0].exist == 1:
                m.mqtt_client.publish('trafficlight',self.cars[0].name+str(self.cars[0].ID)+':no')
                self.cars[0].exist=0
                flag = 1

        # send Traffic lights
        if self.traffic_light == 'red':
            obpos = self.length
            # print(self.cars[0].name)
            if self.cars[0].traffic_light != 'red':
                m.mqtt_client.publish('trafficlight',self.cars[0].name+str(self.cars[0].ID)+':red')
                self.cars[0].traffic_light = 'red'
        elif self.traffic_light == 'yellow':
            if self.cars[0].traffic_light != 'yellow':
                m.mqtt_client.publish('trafficlight',self.cars[0].name+str(self.cars[0].ID)+':yellow')
                self.cars[0].traffic_light = 'yellow'
            if car.pos + car.safety_distance < self.length:
                obpos = self.length
        elif self.cars[0].traffic_light == 'green':
            if self.cars[0].traffic_light != 'green':
                m.mqtt_client.publish('trafficlight',self.cars[0].name+str(self.cars[0].ID)+':green')
                self.cars[0].traffic_light = 'green'

        next_car_pos, _ = self.next_car(car)
        #print( str(next_car_pos) +"                 "+ str(car.pos) )
        if next_car_pos and next_car_pos < obpos:
            obpos = next_car_pos
        return obpos

    def reach(self, car):     
        self.cars.insert(0, car)

    def leave(self, car):    #小车消失
        self.cars.remove(car)

class LinearWay(Way):
    def __init__(self, x0, y0, x1, y1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.x0 = x0
        self.y0 = y0
        self.dx = x1 - x0
        self.dy = y1 - y0
        self.length = hypot(x1 - x0, y1 - y0) # m

    x1 = property(lambda self: self.x0 + self.dx)
    y1 = property(lambda self: self.y0 + self.dy)

    def position_car(self, car):
        car.x = self.x0 + self.dx * (car.pos / self.length)
        car.y = self.y0 + self.dy * (car.pos / self.length)

class BezierWay(Way):
    def __init__(self, x0, y0, x1, y1, x2, y2, x3, y3, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.x3 = x3
        self.y3 = y3

        self.length = 0.
        xp = self.x0
        yp = self.y0
        for i in range(1, 65):
            t = float(i) / 64.
            u = 1. - t
            x = u*u*u*self.x0 + \
             3.*u*u*t*self.x1 + \
             3.*u*t*t*self.x2 + \
                t*t*t*self.x3
            y = u*u*u*self.y0 + \
             3.*u*u*t*self.y1 + \
             3.*u*t*t*self.y2 + \
                t*t*t*self.y3
            self.length += hypot(x - xp, y - yp)
            xp = x
            yp = y

    def position_car(self, car):
        t = car.pos / self.length
        u = 1. - t
        car.x = u*u*u*self.x0 + \
             3.*u*u*t*self.x1 + \
             3.*u*t*t*self.x2 + \
                t*t*t*self.x3
        car.y = u*u*u*self.y0 + \
             3.*u*u*t*self.y1 + \
             3.*u*t*t*self.y2 + \
                t*t*t*self.y3

class Vehicle:
    length = 4.
    acceleration = 7.84 / 2.
    deceleration = 7.84 # Max possible with g=9.81m/s, frictioncoeff=.8
    traffic_light ='green'
    exist = 1
    ID = 0

    def __init__(self, way):
        self.pos = 0. # m along current way
        self.speed = 0. # m/s
        self.speed_mul = 1.0 - 0.3 * random() # % of speedlimit this car reaches

        self.way = way
        self.x = self.xp = way.x0
        self.y = self.yp = way.y0

    def update(self, dt):
        if self.pos > self.way.length:
            self.way.leave(self)
            self.pos -= self.way.length
            #for obj in self.way.to :
              #  print(obj)
            if self.way.to:
                #print('#################################'+ str(self.pos) )
                self.way = choice(self.way.to)
                #print( str(self.way.to[0].x1) +"                   " + str(self.way.to[0].y1) )
                self.way.reach(self)
            else:
                #print('***********************************************')
                self.on_dead_end()

        w = self.way

        # Reach top speed / Decelerate to top speed
        if self.speed < w.speed_limit * self.speed_mul:
            acc = self.acceleration
        else:
            acc = 0.

        # Keep safe distance from obstacles(cars, traffic stops...)
        obstacle_pos = w.next_obstacle_position(self)
        obstacle_dist = obstacle_pos - self.pos
        if obstacle_dist < self.safety_distance + self.length * 1.5:
            acc = -self.deceleration

        self.speed += acc * dt
        if self.speed < 0.:
            self.speed = 0.
        self.pos += self.speed * dt
        #print("Vehicle : "+  str(self.pos)  +'   '+   str(self.way.length)  )
        #print('>>>>>>>>>>>>>8888888888888888>>>>>>>>>>>>>>')
        #print(self.pos)
        #print('>>>>>>>>>>>>>8888888888888888>>>>>>>>>>>>>>')

        # update coordinates
        self.xp = self.x
        self.yp = self.y
        self.way.position_car(self)

    @property
    def safety_distance(self):
        t = self.speed / self.deceleration  # Braking time
        return self.speed * t + self.deceleration * t * t

    def on_dead_end(self):
        """Called when leaving a dead end Way. e.g.: to destroy self."""
        pass


class mqttThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.mqtt_on_connect
            self.mqtt_client.on_message = self.mqtt_on_message
            self.message=''
            self.mqtt_client.connect('localhost', 1883, 60)

            
        except socket.gaierror:
            print ('No Connection')
        #self.fallbackLoopTime = fallbackLoopTime
        
    def stopped(self):
        return self.event.isSet()

    def stop(self):
        self.event.set()

    def mqtt_on_connect(self,client, userdata,flags, rc):
        self.mqtt_client.subscribe('addcar')
        self.mqtt_client.subscribe('get_pos')
        self.mqtt_client.subscribe('trafficlight')


    def mqtt_on_message(self,client, userdata, msg):
        Str = str(msg.payload)
        self.message = Str[11]
        print(self.message)
        # if message == "b'1'":
        #     print('YES')

    # def pos_publish(pos):
    #     self.client.publish("get_pos",pos)

    def run(self):
        self.mqtt_client.loop_start()
        #self.event.wait(self.interval)


m = mqttThread()
m.run()
