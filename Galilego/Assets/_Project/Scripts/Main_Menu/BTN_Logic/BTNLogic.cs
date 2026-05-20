using UnityEngine;
using UnityEngine.EventSystems;

public class BTNLogic : MonoBehaviour
{
    [SerializeField] private BTNEventHandler newGameBtn;
    [SerializeField] private BTNEventHandler settingBtn;
    [SerializeField] private BTNEventHandler exitBtn;
    

    void Start()
    {
        newGameBtn.OnClick += HandleClick;
        exitBtn.OnClick += HandleClick;
        settingBtn.OnClick += HandleClick;
    }

    private void HandleClick(BTNEventHandler btn)
    {
        switch (btn.gameObject.name)
        {
            case "New Game":
                Debug.Log("Начало игры");
                CoreSceneManager.Instance.LoadNextScene();
                break;

            case "Setings":
                Debug.Log("Открытие настроек");
                PanelManager.Instance.OpenSettingPanel();
                break;  

            case "Exit_BTN":
                Debug.Log("Закрытие игры");
                break;
        }
    }
}
