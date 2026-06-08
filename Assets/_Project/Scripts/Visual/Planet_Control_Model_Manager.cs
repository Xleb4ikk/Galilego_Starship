using Galilego.MoonVisualSetting;
using Galilego.PlanetModelInfo;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Planet_Control_Model_Manager : MonoBehaviour
{
    [SerializeField] private List<PlanetModels> PlanetModels = new List<PlanetModels>();

    public int quality = 0;

    void Start()
    {
        if (SaveManager.Settings == null)
            Debug.LogWarning("Настройки будут работать только после того если игру запустить через меню");
        else
            SetModelQuality(SaveManager.Settings.TextureQuality);
    }

    public void SetModelQuality(int quality)
    {

        for (int i = 0; i < PlanetModels.Count; i++)
        {
            PlanetModels Planet = PlanetModels[i];

            if (Planet == null)
                continue;

            Planet.HighQualityModel.SetActive(quality == 0);
            Planet.MidQualityModel.SetActive(quality == 1);
            Planet.LowQualityModel.SetActive(quality == 2);
        }
    }
}
