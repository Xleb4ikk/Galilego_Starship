using UnityEngine;

public class SettingsController : MonoBehaviour
{
    public void SetResolution(int Value)
    {
        PlayerPrefs.SetInt("Resolution", Value);
    }

    public void SetFrameLimit(int Value)
    {
        PlayerPrefs.SetInt("FrameLimit", Value);
    }

    public void SetTexureQuality(int Value)
    {
        PlayerPrefs.SetInt("TextureQuality", Value);
    }

    public void SetShadowQuality(int Value)
    {
        PlayerPrefs.SetInt("ShadowQuality", Value);
    }

    public void SetEffectsQuality(int Value)
    {
        PlayerPrefs.SetInt("EffectsQuality", Value);
    }

    public void SetAntiAliasing(int Value)
    {
        PlayerPrefs.SetInt("AntuAliasing", Value);
    }

    public void SetMasterVolume(float Value)
    {
        PlayerPrefs.SetFloat("MasterVolume", Value);
    }

    public void SetMusicVolume(float Value)
    {
        PlayerPrefs.SetFloat("MusicVolume", Value);
    }

    public void SetSFXVolume(float Value)
    {
        PlayerPrefs.SetFloat("SFXVolume", Value);
    }

    public void SavePlayerPrefs()
    {
        PlayerPrefs.Save();
    }

    public void DeleteAllPlayerPrefs()
    {
        PlayerPrefs.DeleteAll();
    }
}
