/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections.Generic;
using UnityEngine;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    public class WaypointsServiceProvider : UnityServiceProvider<WaypointsRequest, WaypointsResponse>
    {

        protected override void Start() {
            base.Start();
        }

        protected override bool ServiceCallHandler(WaypointsRequest request, out WaypointsResponse response)
        {
            response = new WaypointsResponse();            
            Vector2[] waypoints_raw = GameManager.instance.GetWaypoints();

            response.waypoints = new Gps[waypoints_raw.Length];
            
            for (int i=0 ;i<waypoints_raw.Length; i++) {
                Gps pt = new Gps();
                pt.latitude = (waypoints_raw[i].y) / 110944.33 + 35.205853f;
                pt.longitude = (waypoints_raw[i].x) / 91058.93 + -97.442325f;
                response.waypoints[i] = pt;
            }

            return true;
        }
    }
}