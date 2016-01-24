#!/usr/bin/env python
import web
import rospy
import std_msgs.msg as msg

urls = ("/","DataLog")

app = web.application(urls, globals())

class DataLog:
    def __init__(self):
        self.data = {"Temperature":"32 F",
                     "Vehicle Ground Speed":"20 m/s",
                     "Vehicle Aerial Speed":"23.45m/s",
                     "Vehicle Altitude":"10.765m",
                    "Plant Himidity": "30g/cubic.metre",
                     "Soil pH":6.5}
        self.reader = DataListener()

    def GET(self):
        render = web.template.render('templates')
        return render.display(self.data)

    def refresh_data(self):
        if self.reader.updated:
            self.data = self.reader.get_data()

class DataListener:
    def __init__(self):
        self.data = None
        self.updated = False
        self.sub = rospy.Subscriber('sensor', msg.Float64, self._data_callback)

    @property
    def get_data(self):
        self.updated = False
        return self.data

    def _data_callback(self, reply):
        self.data = reply.data
        self.updated = True

if __name__ == "__main__":
    app.run()
