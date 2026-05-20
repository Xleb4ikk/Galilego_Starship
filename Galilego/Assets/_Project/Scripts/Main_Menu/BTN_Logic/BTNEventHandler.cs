using System;
using UnityEngine;
using UnityEngine.EventSystems;

public class BTNEventHandler : MonoBehaviour, IPointerEnterHandler,
    IPointerExitHandler,
    IPointerClickHandler
{
    public Action<BTNEventHandler> OnClick;
    public Action<BTNEventHandler> OnHoverEnter;
    public Action<BTNEventHandler> OnHoverExit;

    void Start()
    {
        DebugManager.Log($"Start BTNEventHandler: {this}");
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        OnHoverEnter?.Invoke(this);
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        OnHoverExit?.Invoke(this);
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        OnClick?.Invoke(this);
    }
}
