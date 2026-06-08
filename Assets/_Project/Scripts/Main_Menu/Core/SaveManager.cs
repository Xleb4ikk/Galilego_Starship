using System;
using System.IO;
using Unity.VisualScripting;
using UnityEngine;

public class GameSettings
{
    public bool Fullscreen = true;
    public bool VSync = true;
    public bool Motionblur = true;

    public int With = 1920;
    public int Height = 1080;
    public int FrameLimit = 100;

    public int Quality = 0;
    public int TextureQuality = 0;
    public int ShadowQuality = 0;
    public int EffectsQuality = 0;
    public int AntiAliasing = 0;

    public float MasterVolume = 1.0f;
    public float MusicVolume = 1.0f;
    public float SFXVolume = 1.0f;
}

public class SaveManager : MonoBehaviour
{
    private void Awake()
    {
        Load();
    }

    private static string SaveDirectory =>
        Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            Application.productName
        );

    private static string SavePath =>
        Path.Combine(SaveDirectory, "settings.json");

    public static GameSettings Settings {  get; private set; }

    public static void Load()
    {

        Directory.CreateDirectory(SaveDirectory);

        if (File.Exists(SavePath))
        {
            string json = File.ReadAllText(SavePath);
            Settings = JsonUtility.FromJson<GameSettings>(json);
        }
        else
        {
            Settings = new GameSettings();
            Save();
        }
    }

    public static void Save()
    {
        Directory.CreateDirectory(SaveDirectory);

        string json = JsonUtility.ToJson(Settings, true);
        File.WriteAllText(SavePath, json);
    }
}
