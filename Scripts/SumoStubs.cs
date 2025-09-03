#if !SUMO_INTEGRATION
using UnityEngine;

// Lightweight stubs so the project can compile without the real SUMO integration.
// Define SUMO_INTEGRATION in Player Settings > Scripting Define Symbols when you add the real plugin.

public class SumoSocketClient : MonoBehaviour
{
    // Put any minimal fields you may need for tests here.
}

namespace tum_car_controller
{
    public partial class CarController
    {
        // Minimal tuple-like struct to avoid tuple deconstruction issues on older compilers if needed
        private struct ControlResult
        {
            public float steeringValue;
            public float torqueInput;
            public float desiredVelocity;
        }

        private bool SumoVehicleDetect(ref SumoSocketClient sock, string vehicleId)
        {
            // Always true for standalone mode; replace with real detection when integrated.
            return true;
        }

        private Rigidbody SumoTaxiTeleport(
            ref SumoSocketClient sock,
            string vehicleId,
            Rigidbody rb,
            float steeringGain,
            ref PIDController pidSpeed,
            ref PIDController pidDist,
            ref Vector2 lookAheadMarker)
        {
            // In standalone mode, just keep the RB where it is and set look-ahead in front
            if (rb != null)
            {
                var forward = rb.transform.forward;
                var pos = rb.position + forward * 2f;
                lookAheadMarker = new Vector2(pos.x, pos.z);
            }
            return rb;
        }

        private (float steeringValue, float torqueInput, float desiredVelocity) SumoVehicleControl(
            ref SumoSocketClient sock,
            string vehicleId,
            Rigidbody rb,
            float steeringGain,
            ref PIDController pidSpeed,
            ref PIDController pidDist,
            ref Vector2 lookAheadMarker)
        {
            // Simple placeholder logic: aim straight ahead with zero torque and a modest desired speed
            float desiredVelocity = 5f; // m/s
            float steeringValue = 0f;
            float torqueInput = 0.5f; // light acceleration

            if (rb != null)
            {
                var forward = rb.transform.forward;
                var pos = rb.position + forward * 5f;
                lookAheadMarker = new Vector2(pos.x, pos.z);
            }

            return (steeringValue, torqueInput, desiredVelocity);
        }

        private float getVehicleStopState(ref SumoSocketClient sock, string vehicleId)
        {
            // 0 means not stopped in this stub
            return 0f;
        }
    }
}
#endif
