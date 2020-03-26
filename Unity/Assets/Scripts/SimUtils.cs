using UnityEngine;

public static class SimUtils
{
    public static float getRandNormal(float mean, float stdDev)
    {
        float u1 = 1.0f - Random.value; //uniform(0,1] random doubles
        float u2 = 1.0f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                        Mathf.Sin(2.0f * Mathf.PI * u2); //random normal(0,1)

        return stdDev * randStdNormal;
    }
}
