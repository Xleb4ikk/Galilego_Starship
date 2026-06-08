using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class UIVisualControler : MonoBehaviour
{
    [Header("Dropdown")]
    [SerializeField] private TMP_Dropdown ResolutionDropDown;
    [SerializeField] private TMP_Dropdown FrameLimitDropDown;
    [SerializeField] private TMP_Dropdown TextureQualityDropDown;
    [SerializeField] private TMP_Dropdown ShadowQualityDropDown;
    [SerializeField] private TMP_Dropdown EffectsQualityDropDown;
    [SerializeField] private TMP_Dropdown Anti_AliasingDropDown;

    [Header("SliderText")]
    [SerializeField] private TMP_Text MasterVolumeProcentText;
    [SerializeField] private TMP_Text MusicVolumeProcentText;
    [SerializeField] private TMP_Text SFXVolumeProcentText;

    [Header("Slider")]
    [SerializeField] private Slider MasterVolumeslider;
    [SerializeField] private Slider MusicVolumeslider;
    [SerializeField] private Slider SFXVolumeslider;

    private void Start()
    {
        InitializeUIFromSettings();
    }

    private void InitializeUIFromSettings()
    {
        OnChangeProcentMasterVolume(SaveManager.Settings.MasterVolume);
        OnChangeProcentMusicVolume(SaveManager.Settings.MusicVolume);
        OnChangeProcentSFXVolume(SaveManager.Settings.SFXVolume);
        OnChangeMasterVolumeSlider(SaveManager.Settings.MasterVolume);
        OnChangeMusicVolumSlider(SaveManager.Settings.MusicVolume);
        OnChangeSFXVolumeslider(SaveManager.Settings.SFXVolume);
        //OnChangeResolutionDropDown(SaveManager.Settings.);
        OnChangeFrameLimitDropDown(SaveManager.Settings.FrameLimit);
        OnChangeTextureQualityDropDown(SaveManager.Settings.TextureQuality);
        OnChangeShadowQualityDropDown(SaveManager.Settings.ShadowQuality);
        OnChangeEffectsQualityDropDown(SaveManager.Settings.EffectsQuality);
        OnChangeAnti_AliasingDropDown(SaveManager.Settings.AntiAliasing);
    }

    //Sliders procent
    public void OnChangeProcentMasterVolume(float volume) 
        => MasterVolumeProcentText.text = $"{(int)volume}%";

    public void OnChangeProcentMusicVolume(float volume) 
        => MusicVolumeProcentText.text = $"{(int)volume}%";

    public void OnChangeProcentSFXVolume(float volume) 
        => SFXVolumeProcentText.text = $"{(int)volume}%";

    //Sliders 
    public void OnChangeMasterVolumeSlider(float volume) 
        => MasterVolumeslider.value = volume;

    public void OnChangeMusicVolumSlider(float volume) 
        => MusicVolumeslider.value = volume;

    public void OnChangeSFXVolumeslider(float volume)
        => SFXVolumeslider.value = volume;

    //DropDown
    public void OnChangeResolutionDropDown(int value) 
        => ResolutionDropDown.value = value;

    public void OnChangeFrameLimitDropDown(int value)
        => FrameLimitDropDown.value = value;

    public void OnChangeTextureQualityDropDown(int value)
        => TextureQualityDropDown.value = value;

    public void OnChangeShadowQualityDropDown(int value)
        => ShadowQualityDropDown.value = value;

    public void OnChangeEffectsQualityDropDown(int value)
        => EffectsQualityDropDown.value = value;

    public void OnChangeAnti_AliasingDropDown(int value)
        => Anti_AliasingDropDown.value = value;
}
