Shader "UI/ButtonSweep"
{
    Properties
    {
        [PerRendererData] _MainTex ("Sprite Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1,1,1,1)
        
        _SweepColor ("Sweep Color", Color) = (1,1,1,1)
        _SweepPosition ("Sweep Position", Range(-0.5, 1.5)) = -0.5
        _SweepWidth ("Sweep Width", Range(0.05, 0.6)) = 0.25
        _SweepIntensity ("Sweep Intensity", Range(0, 2)) = 1.2
    }

    SubShader
    {
        Tags { "Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent" "PreviewType"="Plane" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata_t
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float2 texcoord : TEXCOORD0;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                fixed4 color : COLOR;
                float2 texcoord : TEXCOORD0;
            };

            sampler2D _MainTex;
            fixed4 _Color;
            fixed4 _SweepColor;
            float _SweepPosition;
            float _SweepWidth;
            float _SweepIntensity;

            v2f vert (appdata_t v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.texcoord = v.texcoord;
                o.color = v.color * _Color;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.texcoord) * i.color;
                
                // Красивый горизонтальный блик
                float dist = i.texcoord.x - _SweepPosition;
                float sweep = smoothstep(0, _SweepWidth, dist) * (1 - smoothstep(_SweepWidth, _SweepWidth * 2, dist));
                sweep = pow(sweep, 0.6); // делает блик мягче и красивее
                
                col.rgb += sweep * _SweepColor.rgb * _SweepIntensity;
                
                return col;
            }
            ENDCG
        }
    }
}