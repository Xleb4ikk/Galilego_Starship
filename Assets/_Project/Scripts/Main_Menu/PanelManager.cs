using UnityEngine;
using DG.Tweening;

public class PanelManager : MonoBehaviour
{
    public static PanelManager Instance;

    [SerializeField] private Transform Panel;

    [SerializeField] private float Move = 0f;
    [SerializeField] private float CloseMove = 0f;

    [SerializeField] private float Time = 0f;

    private void Awake()
    {
        Debug.Log("PanelManager created");

        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void Start()
    {
        Panel.DOMoveX(-686.41f, 0);
    }

    public void OpenSettingPanel()
    {
        Panel.DOMoveX(Move, Time)
            .SetEase(Ease.InOutQuad)
            .OnComplete(() => DebugManager.Log("Открытие завершено"));
    }

    public void CloseSettingPanel()
    {
        Panel.DOMoveX(CloseMove, Time)
            .SetEase(Ease.InOutQuad)
            .OnComplete(() => DebugManager.Log("Закрытие завершено"));
    }
}
