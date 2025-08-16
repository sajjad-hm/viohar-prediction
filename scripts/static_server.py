#!/usr/bin/env python
# Python 2.7
import rospy
import os
import threading
import signal
from SimpleHTTPServer import SimpleHTTPRequestHandler
from SocketServer import TCPServer

class HTTPThread(threading.Thread):
    def __init__(self, web_dir, port):
        super(HTTPThread, self).__init__()
        self.web_dir = web_dir
        self.port = port
        self.httpd = None

    def run(self):
        cwd = os.getcwd()
        try:
            os.chdir(self.web_dir)
            self.httpd = TCPServer(("", self.port), SimpleHTTPRequestHandler)
            rospy.loginfo("Serving %s at http://localhost:%d", self.web_dir, self.port)
            self.httpd.serve_forever()
        finally:
            os.chdir(cwd)

    def stop(self):
        if self.httpd:
            self.httpd.shutdown()

def main():
    rospy.init_node('viohar_static_server', anonymous=False)
    web_dir = rospy.get_param('~web_dir', os.path.join(os.path.dirname(__file__), '..', 'web'))
    port = int(rospy.get_param('~port', 8000))

    server = HTTPThread(web_dir, port)
    server.daemon = True
    server.start()

    def on_shutdown():
        rospy.loginfo("Shutting down static server...")
        server.stop()
    rospy.on_shutdown(on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    main()
