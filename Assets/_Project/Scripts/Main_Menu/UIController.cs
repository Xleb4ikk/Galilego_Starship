using System;
using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class UIController : MonoBehaviour
{
    [Header("Button")]
    [SerializeField] private EventHandler newGameBtn;
    [SerializeField] private EventHandler settingBtn;
    [SerializeField] private EventHandler exitBtn;
    [SerializeField] private EventHandler ApplyButtonSettings;
    [SerializeField] private EventHandler ResetButtonSettings;
    [SerializeField] private EventHandler CloseSettings;

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
    [SerializeField] private SaveSettingsControler saveSettingsControler;

    [Header("UiObject")]
    [SerializeField] private GameObject MainMenu_Ui;
    [SerializeField] private GameObject Button;
    [SerializeField] private GameObject SettingsPanel;

    private bool isOpenMenu = false;
    private bool isInitialized = false;

    void Update()
    {
        if (SceneManager.GetActiveScene().name != "JupiterMain")
            return;

        if (!isInitialized)
        {
            Button.SetActive(false);
            SettingsPanel.SetActive(false);
            isOpenMenu = false;

            isInitialized = true;
        }

        if (Keyboard.current.escapeKey.wasPressedThisFrame)
        {
            isOpenMenu = !isOpenMenu;

            Button.SetActive(isOpenMenu);
            SettingsPanel.SetActive(isOpenMenu);
        }
    }

    private void Awake() 
        => DontDestroyOnLoad(MainMenu_Ui);

    private void OnEnable() 
        => Subscribe();

    private void OnDisable() 
        => Unsubscribe();

    private void Subscribe()
    {
        //Подписка на кнопки
        newGameBtn.OnClick += OnNewGame;
        settingBtn.OnClick += OnSettings;
        exitBtn.OnClick += OnExit;
        ApplyButtonSettings.OnClick += OnApplySettings;
        ResetButtonSettings.OnClick += OnResetSettings;
        CloseSettings.OnClick += OnCloseMenuSettings;

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

    private void Unsubscribe()
    {
        //Отписка на кнопки
        newGameBtn.OnClick -= OnNewGame;
        settingBtn.OnClick -= OnSettings;
        exitBtn.OnClick -= OnExit;
        ApplyButtonSettings.OnClick -= OnApplySettings;
        ResetButtonSettings.OnClick -= OnResetSettings;
        CloseSettings.OnClick -= OnCloseMenuSettings;

        ResolutionDropDown.OnDropdownValueChanged -= OnChangeResolution;
        FrameLimitDropDown.OnDropdownValueChanged -= OnChangeFrameLimit;
        TextureQualityDropDown.OnDropdownValueChanged -= OnChangeTextureQuality;
        ShadowQualityDropDown.OnDropdownValueChanged -= OnChangeShadowQuality;
        EffectsQualityDropDown.OnDropdownValueChanged -= OnChangeEffectsQuality;
        Anti_AliasingDropDown.OnDropdownValueChanged -= OnChangeAnti_Aliasing;
        MasterVolumeSlider.OnSliderValueChanged -= OnChangeMasterVolume;
        MusicVolumeSlider.OnSliderValueChanged -= OnChangeMusicVolume;
        SFXVolumeSlider.OnSliderValueChanged -= OnChangeSFXVolume;
    }

    //Кнопки
    private void OnNewGame(EventHandler btn)
    {
        CoreSceneManager.Instance.LoadNextScene();
    }

    private void OnSettings(EventHandler btn)
    {
        PanelManager.Instance.OpenSettingPanel();
    }

    private void OnExit(EventHandler btn)
    {
        Application.Quit();
    }

    private void OnApplySettings(EventHandler btn) 
        => saveSettingsControler.SavePlayerPrefs();

    private void OnResetSettings(EventHandler btn) 
        => saveSettingsControler.DeleteAllPlayerPrefs();

    private void OnCloseMenuSettings(EventHandler btn) 
        => PanelManager.Instance.CloseSettingPanel();

    //Списки
    private void OnChangeResolution(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeResolution: " + Value);
        saveSettingsControler.SetResolution(Value);
    }

    private void OnChangeFrameLimit(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeFrameLimit: " + Value);
        saveSettingsControler.SetFrameLimit(Value);
    }

    private void OnChangeTextureQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeTextureQuality: " + Value);
        saveSettingsControler.SetTexureQuality(Value);
    }

    private void OnChangeShadowQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeShadowQuality: " + Value);
        saveSettingsControler.SetShadowQuality(Value);
    }

    private void OnChangeEffectsQuality(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeEffectsQualityy: " + Value);
        saveSettingsControler.SetEffectsQuality(Value);
    }

    private void OnChangeAnti_Aliasing(EventHandler Dropdown, int Value)
    {
        Debug.Log("ChangeAnti_Aliasing: " + Value);
        saveSettingsControler.SetAntiAliasing(Value);
    }

    //Слайдеры
    private void OnChangeMasterVolume(EventHandler Slider, float Value)
    {
        Debug.Log("MasterVolume " + Value);

        saveSettingsControler?.SetMasterVolume(Value);
        UIVisualContoller.OnChangeProcentMasterVolume(Value);
    }

    private void OnChangeMusicVolume(EventHandler Slider, float Value)
    {
        Debug.Log("MusicVolume" + Value);
        saveSettingsControler?.SetMusicVolume(Value);
        UIVisualContoller.OnChangeProcentMusicVolume(Value);
    }

    private void OnChangeSFXVolume(EventHandler Slider, float Value)
    {
        Debug.Log("SFXVolume" + Value);
        saveSettingsControler.SetSFXVolume(Value);
        UIVisualContoller.OnChangeProcentSFXVolume(Value);
    }
}
