using UnityEngine;
using UnityEngine.SceneManagement;

public class CoreSceneManager : MonoBehaviour
{
    public static CoreSceneManager Instance;

    private void Awake()
    {
        Debug.Log("CoreSceneManager created");

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

    public void LoadSceneWithName(string sceneName)
    {
        try
        {
            DebugManager.Log($"Загрузка сцены по имени: {sceneName}");
            SceneManager.LoadScene(sceneName);
        }
        catch (System.Exception e)
        {
            DebugManager.Error($"Ошибка загрузки сцены по имени: {e}");
        }
    }

    public void LoadSceneWithIndex(int index)
    {
        try
        {
            if (index >= 0 && index < SceneManager.sceneCountInBuildSettings)
            {
                DebugManager.Log($"Загрузка сцены по индексу: {index}");
                SceneManager.LoadScene(index);
            }
            else
            {
                DebugManager.Warning($"Сцена с индексом {index} не существует");
            }
        }
        catch (System.Exception e)
        {
            DebugManager.Error($"Ошибка загрузки сцены по индексу: {e}");
        }
    }

    public void LoadNextScene()
    {
        int index = SceneManager.GetActiveScene().buildIndex + 1;

        if (index < SceneManager.sceneCountInBuildSettings)
        {
            DebugManager.Log($"Загрузка следующей сцены: {index}");
            SceneManager.LoadScene(index);
        }
        else
        {
            DebugManager.Warning("Это последняя сцена");
        }
    }

    public void LoadPreviousScene()
    {
        int index = SceneManager.GetActiveScene().buildIndex - 1;

        if (index >= 0)
        {
            DebugManager.Log($"Загрузка предыдущей сцены: {index}");
            SceneManager.LoadScene(index);
        }
        else
        {
            DebugManager.Warning("Это первая сцена");
        }
    }

    public void ReloadCurrentScene()
    {
        int index = SceneManager.GetActiveScene().buildIndex;

        DebugManager.Log($"Перезагрузка сцены: {index}");
        SceneManager.LoadScene(index);
    }

    public int GetCurrentSceneIndex()
    {
        return SceneManager.GetActiveScene().buildIndex;
    }

    public string GetCurrentSceneName()
    {
        return SceneManager.GetActiveScene().name;
    }
}