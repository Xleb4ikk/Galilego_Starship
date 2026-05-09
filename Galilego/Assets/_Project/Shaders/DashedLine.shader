Shader "Custom/DashedLine"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _DashSize ("Dash Size", Float) = 1
        _GapSize ("Gap Size", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" "RenderPipeline" = "UniversalPipeline" }
        LOD 100

        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            float4 _Color;
            float _DashSize;
            float _GapSize;

            Varyings vert (Attributes input)
            {
                Varyings output;
                output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
                output.uv = input.uv;
                output.color = input.color;
                return output;
            }

            half4 frag (Varyings input) : SV_Target
            {
                float totalSize = _DashSize + _GapSize;
                float pos = input.uv.x * 100; // Multiply by some factor for density
                if (fmod(pos, totalSize) > _DashSize)
                {
                    discard;
                }
                return _Color * input.color;
            }
            ENDHLSL
        }
    }
}