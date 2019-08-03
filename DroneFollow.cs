using UnityEngine;
using ROSBridgeLib.sensor_msgs;
using System;

public class DroneFollow : MonoBehaviour
{
    public DJI_SDK dji_sdk;

    NavSatFixMsg origin_position = null;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (dji_sdk == null)
        {
            dji_sdk = GetComponent<DJI_SDK>();
        }
        if (dji_sdk != null && dji_sdk.sdk_ready)
        {

            if (origin_position == null)
            {
                origin_position = dji_sdk.GetGPSPosition();
            } else
            {
                Vector3 ground_displacement = DJI_SDK.VectorFromGPS(origin_position, dji_sdk.GetGPSPosition());
                transform.localPosition = new Vector3(ground_displacement.x, dji_sdk.GetHeightAboveTakeoff(), ground_displacement.y);
            }
            transform.localRotation = dji_sdk.GetAttitude();

            //if (Input.GetKeyUp(KeyCode.W))
            //{
            //    dji_sdk.PublishRelativeSetPoint(new Vector2(0, 1), dji_sdk.GetHeightAboveTakeoff(), 0);
            //} else if (Input.GetKeyUp(KeyCode.A))
            //{
            //    dji_sdk.PublishRelativeSetPoint(new Vector2(-1, 0), dji_sdk.GetHeightAboveTakeoff(), 0);
            //} else if (Input.GetKeyUp(KeyCode.S))
            //{
            //    dji_sdk.PublishRelativeSetPoint(new Vector2(0, -1), dji_sdk.GetHeightAboveTakeoff(), 0);
            //} else if (Input.GetKeyUp(KeyCode.D))
            //{
            //    dji_sdk.PublishRelativeSetPoint(new Vector2(1, 0), dji_sdk.GetHeightAboveTakeoff(), 0);
            //}

            if (Input.GetKey(KeyCode.W))
            {
                dji_sdk.PublishRateSetpoint(0, 5, dji_sdk.GetHeightAboveTakeoff(), 0);
            } else if (Input.GetKey(KeyCode.A))
            {
                dji_sdk.PublishRateSetpoint(-5, 0, dji_sdk.GetHeightAboveTakeoff(), 0);
            } else if (Input.GetKey(KeyCode.S))
            {
                dji_sdk.PublishRateSetpoint(0, -5, dji_sdk.GetHeightAboveTakeoff(), 0);
            } else if (Input.GetKey(KeyCode.D))
            {
                dji_sdk.PublishRateSetpoint(5, 0, dji_sdk.GetHeightAboveTakeoff(), 0);
            } else
            {
                dji_sdk.PublishRateSetpoint(0, 0, dji_sdk.GetHeightAboveTakeoff(), 0);
            }

            if (Input.GetKeyUp(KeyCode.Space))
            {
                if (!dji_sdk.HasAuthority())
                {
                    dji_sdk.SetSDKControl(true);
                } else if (dji_sdk.GetFlightStatusM100() == DJI_SDK.FlightStatusM100.ON_GROUND_STANDBY)
                {
                    dji_sdk.ExecuteTask(DJI_SDK.DroneTask.TAKEOFF);
                } else if (dji_sdk.GetFlightStatusM100() == DJI_SDK.FlightStatusM100.IN_AIR_STANDBY)
                {
                    dji_sdk.ExecuteTask(DJI_SDK.DroneTask.LAND);
                }
            } else if (Input.GetKeyUp(KeyCode.Escape))
            {
                if (dji_sdk.HasAuthority())
                {
                    dji_sdk.ExecuteTask(DJI_SDK.DroneTask.GO_HOME);
                }
            } else if (Input.GetKeyUp(KeyCode.R))
            {
                if (dji_sdk.HasAuthority())
                {
                    dji_sdk.SetSDKControl(false);
                }
            }
        }
    }
}
