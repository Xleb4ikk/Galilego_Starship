using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;

public class UIController : MonoBehaviour
{
    [Header("Button")]
    [SerializeField] private EventHandler newGameBtn;
    [SerializeField] private EventHandler settingBtn;
    [SerializeField] private EventHandler exitBtn;

    [Header("Dropdown")]
    [SerializeField] private EventHandler ResolutionDropDown;
    [SerializeField] private EventHandler FrameLimitDropDown;
    [SerializeField] private EventHandler TextureQualityDropDown;
    [SerializeField] private EventHandler ShadowQualityDropDown;
    [SerializeField] private EventHandler EffectsQualityDropDown;
    [SerializeField] private EventHandler Anti_AliasingDropDown;

    [Header("Slider")]
    [SerializeField] private EventHandler MasterVolumeSlider;
    [SerializeField] private EventHandler MusicVolumeSlider;
    [SerializeField] private EventHandler SFXVolumeSlider;

    [Header("Scripts")]
    [SerializeField] private UIVisualControler UIVisualContoller;
    [SerializeField] private SettingsController settingsController;


    void Start()
    {
        newGameBtn.OnClick += OnNewGame;
        settingBtn.OnClick += OnSettings;
        exitBtn.OnClick += OnExit;
        ResolutionDropDown.OnDropdownValueChanged += OnChangeResolution;
        FrameLimitDropDown.OnDropdownValueChanged += OnChangeFrameLimit;
        TextureQualityDropDown.OnDropdownValueChanged += OnChangeTextureQuality;
        ShadowQualityDropDown.OnDropdownValueChanged += OnChangeShadowQuality;
        EffectsQualityDropDown.OnDropdownValueChanged += OnChangeEffectsQuality;
        Anti_AliasingDropDown.OnDropdownValueChanged += OnChangeAnti_Aliasing;
        MasterVolumeSlider.OnSliderValueChanged += OnChangeMasterVolume;
        MusicVolumeSlider.OnSliderValueChanged += OnChangeMusicVolume;
        SFXVolumeSlider.OnSliderValueChanged += OnChangeSFXVolume;

    }

    //Кнопки
    private void OnNewGame(EventHandler btn)
    {
        Debug.Log("Начало игры");
        CoreSceneManager.Instance.LoadNextScene();
    }

    private void OnSettings(EventHandler btn)
    {
        Debug.Log("Открытие настроек");
        PanelManager.Instance.OpenSettingPanel();
    }

    private void OnExit(EventHandler btn)
    {
        settingsController.SavePlayerPrefs();
        Debug.Log("Закрытие игры");
    }

    //Списки
    private void OnChangeResolution(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeResolution: " + Value);
        settingsController.SetResolution(Value);
    }

    private void OnChangeFrameLimit(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeFrameLimit: " + Value);
        settingsController.SetFrameLimit(Value);
    }

    private void OnChangeTextureQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeTextureQuality: " + Value);
        settingsController.SetTexureQuality(Value);
    }

    private void OnChangeShadowQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeShadowQuality: " + Value);
        settingsController.SetShadowQuality(Value);
    }

    private void OnChangeEffectsQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeEffectsQualityy: " + Value);
        settingsController.SetEffectsQuality(Value);
    }

    private void OnChangeAnti_Aliasing(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeAnti_Aliasing: " + Value);
        settingsController.SetAntiAliasing(Value);
    }

    //Слайдеры
    private void OnChangeMasterVolume(EventHandler Slider, float Value)
    {
        Debug.Log("MasterVolume" + Value);
        settingsController.SetMasterVolume(Value);
        UIVisualContoller.OnChangeProcentMasterVolume(Value);
    }

    private void OnChangeMusicVolume(EventHandler Slider, float Value)
    {
        Debug.Log("MusicVolume" + Value);
        settingsController?.SetMusicVolume(Value);
        UIVisualContoller.OnChangeProcentMusicVolume(Value);
    }

    private void OnChangeSFXVolume(EventHandler Slider, float Value)
    {
        Debug.Log("SFXVolume" + Value);
        settingsController.SetSFXVolume(Value);
        UIVisualContoller.OnChangeProcentSFXVolume(Value);
    }
}
