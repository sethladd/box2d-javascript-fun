/*
Copyright 2011 Seth Ladd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

importScripts('../Box2D.js');
importScripts('bTest.js');

var Hz = 60;

var box = new bTest(Hz, false);

var loop = function() {
    if (box.ready) {
      box.update();
      var world = box.getState();
      postMessage(world);
    }
}

var intervalId = setInterval(loop, 1000/Hz);

self.onmessage = function(e) {
    switch (e.data.cmd) {
        case 'visible':
            if (intervalId == null) {
                intervalId = setInterval(loop, 1000/Hz);
            }
            break;
        case 'hidden':
            clearInterval(intervalId);
            intervalId = null;
            break;
        case 'bodies':
            box.setBodies(e.data.msg);
            break;
    }
};