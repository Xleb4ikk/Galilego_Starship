using UnityEngine;

public class SaveSettingsControler : MonoBehaviour
{
    public void SetResolution(int Value)
    {
        //PlayerPrefs.SetInt("Resolution", Value);
    }

    public void SetFrameLimit(int Value)
    {
        SaveManager.Settings.FrameLimit = Value;
    }

    public void SetTexureQuality(int Value)
    {
        SaveManager.Settings.TextureQuality = Value;
    }

    public void SetShadowQuality(int Value)
    {
        SaveManager.Settings.ShadowQuality = Value;
    }

    public void SetEffectsQuality(int Value)
    {
        SaveManager.Settings.EffectsQuality = Value;
    }

    public void SetAntiAliasing(int Value)
    {
        SaveManager.Settings.AntiAliasing = Value;
    }

    public void SetMasterVolume(float Value)
    {
        Debug.Log("Settings = " + SaveManager.Settings);
        SaveManager.Settings.MasterVolume = Value;
    }

    public void SetMusicVolume(float Value)
    {
        SaveManager.Settings.MusicVolume = Value;
    }

    public void SetSFXVolume(float Value)
    {
        SaveManager.Settings.SFXVolume = Value;
    }

    public void SavePlayerPrefs()
    {
        SaveManager.Save();
        //PlayerPrefs.Save();
    }

    public void DeleteAllPlayerPrefs()
    {
        PlayerPrefs.DeleteAll();
    }
}
