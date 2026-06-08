using System;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using static UnityEngine.Rendering.DebugUI;

public class EventHandler : MonoBehaviour, IPointerEnterHandler,
    IPointerExitHandler,
    IPointerClickHandler
{
    public Action<EventHandler> OnClick;
    public Action<EventHandler> OnHoverEnter;
    public Action<EventHandler> OnHoverExit;
    public Action<EventHandler, int> OnDropdownValueChanged;
    public Action<EventHandler, float> OnSliderValueChanged;

    private TMP_Dropdown Dropdown;
    private Slider Slider;
    private UnityEngine.UI.Button button;


    private void OnEnable()
    {
        Dropdown = GetComponent<TMP_Dropdown>();
        Slider = GetComponent<Slider>();
        button = GetComponent<UnityEngine.UI.Button>();

        Subscribe();
    }

    private void OnDisable()
    {
        Unsubscribe();
    }

    private void Subscribe()
    {
        if (Dropdown == null && Slider == null && button == null)
            Debug.LogError($"{gameObject.name}: компонент не поддерживается");

        if (Dropdown)
            Dropdown.onValueChanged.AddListener(DropdownChanged);

        if (Slider)
            Slider.onValueChanged.AddListener(OnSliderChanged);
    }

    private void Unsubscribe()
    {
        if (Dropdown)
            Dropdown.onValueChanged.RemoveListener(DropdownChanged);

        if (Slider)
            Slider.onValueChanged.RemoveListener(OnSliderChanged);
    }

    void Start()
    {
#if UNITY_EDITOR
        DebugManager.Log($"Start EventHandler: {this}");
#endif
    }

    public void OnPointerEnter(PointerEventData eventData)
        => OnHoverEnter?.Invoke(this);

    public void OnPointerExit(PointerEventData eventData)
        => OnHoverExit?.Invoke(this);

    public void OnPointerClick(PointerEventData eventData)
        => OnClick?.Invoke(this);

    private void DropdownChanged(int value)
    { 
        OnDropdownValueChanged?.Invoke(this, value);
    } 

    private void OnSliderChanged(float value) => OnSliderValueChanged?.Invoke(this, value);

}
