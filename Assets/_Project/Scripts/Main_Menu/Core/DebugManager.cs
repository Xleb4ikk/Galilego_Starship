using UnityEngine;

public static class DebugManager
{
    public static bool DebugMode = true;

    public static void Log(string Message)
    {
        if (DebugMode)
        {
            Debug.Log(Message);
        }
    }

    public static void Warning(string Message)
    {
        if (DebugMode)
        {
            Debug.LogWarning(Message);
        }
    }

    public static void Error(string Message)
    {
        if (DebugMode)
        {
            Debug.LogError(Message);
        }
    }
}
