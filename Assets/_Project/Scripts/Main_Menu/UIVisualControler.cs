using TMPro;
using UnityEngine;

public class UIVisualControler : MonoBehaviour
{
    [SerializeField] private TMP_Text MasterVolumeProcentText;
    [SerializeField] private TMP_Text MusicVolumeProcentText;
    [SerializeField] private TMP_Text SFXVolumeProcentText;

    public void OnChangeProcentMasterVolume(float volume)
    {
        volume = ((int)volume);
        MasterVolumeProcentText.text = (volume.ToString() + "%");
    }

    public void OnChangeProcentMusicVolume(float volume)
    {
        volume = ((int)volume);
        MusicVolumeProcentText.text = (volume.ToString() + "%");
    }

    public void OnChangeProcentSFXVolume(float volume)
    {
        volume = ((int)volume);
        SFXVolumeProcentText.text = (volume.ToString() + "%");
    }
}
