"""
Unit tests for ProviderManager module.

Tests provider configuration, API key resolution, client initialization,
and fallback functionality.
"""

import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from rob_box_voice.llm.provider_manager import ProviderManager, ProviderConfig


@pytest.fixture
def sample_providers():
    """Sample provider configurations for testing"""
    return {
        "qwen": ProviderConfig(
            name="Qwen (Алибаба)",
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            model="qwen-plus",
            env_var="LLM_API_KEY",
            fallback_env="DASHSCOPE_API_KEY"
        ),
        "deepseek": ProviderConfig(
            name="DeepSeek",
            base_url="https://api.deepseek.com",
            model="deepseek-chat",
            env_var="LLM_API_KEY",
            fallback_env="DEEPSEEK_API_KEY"
        ),
        "openai": ProviderConfig(
            name="OpenAI",
            base_url="https://api.openai.com/v1",
            model="gpt-4",
            env_var="LLM_API_KEY",
            fallback_env="OPENAI_API_KEY"
        )
    }


class TestProviderConfig:
    """Test ProviderConfig dataclass"""
    
    def test_create_provider_config(self):
        """Test creating provider configuration"""
        config = ProviderConfig(
            name="Test Provider",
            base_url="https://test.com/v1",
            model="test-model",
            env_var="TEST_KEY",
            fallback_env="TEST_FALLBACK_KEY"
        )
        
        assert config.name == "Test Provider"
        assert config.base_url == "https://test.com/v1"
        assert config.model == "test-model"
        assert config.env_var == "TEST_KEY"
        assert config.fallback_env == "TEST_FALLBACK_KEY"


class TestProviderManagerInitialization:
    """Test ProviderManager initialization"""
    
    def test_init_with_valid_provider(self, sample_providers):
        """Test initialization with valid default provider"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        assert manager.current_provider == "qwen"
        assert manager.enable_fallback is True
        assert manager.client is None
        assert manager.model is None
    
    def test_init_with_invalid_provider(self, sample_providers):
        """Test initialization with invalid default provider raises error"""
        with pytest.raises(ValueError, match="not in providers"):
            ProviderManager(sample_providers, default_provider="invalid")
    
    def test_init_with_custom_timeouts(self, sample_providers):
        """Test initialization with custom timeout values"""
        manager = ProviderManager(
            sample_providers,
            timeout_total=120.0,
            timeout_connect=20.0
        )
        
        assert manager.timeout_total == 120.0
        assert manager.timeout_connect == 20.0
    
    def test_init_with_fallback_disabled(self, sample_providers):
        """Test initialization with fallback disabled"""
        manager = ProviderManager(
            sample_providers,
            enable_fallback=False
        )
        
        assert manager.enable_fallback is False


class TestProviderRetrieval:
    """Test provider configuration retrieval"""
    
    def test_get_current_provider_config(self, sample_providers):
        """Test getting current provider configuration"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        config = manager.get_provider_config()
        
        assert config.name == "Qwen (Алибаба)"
        assert config.model == "qwen-plus"
    
    def test_get_specific_provider_config(self, sample_providers):
        """Test getting specific provider configuration"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        config = manager.get_provider_config("deepseek")
        
        assert config.name == "DeepSeek"
        assert config.model == "deepseek-chat"
    
    def test_get_invalid_provider_config(self, sample_providers):
        """Test getting invalid provider raises error"""
        manager = ProviderManager(sample_providers)
        
        with pytest.raises(ValueError, match="Unknown provider"):
            manager.get_provider_config("invalid")


class TestAPIKeyResolution:
    """Test API key resolution from multiple sources"""
    
    def test_resolve_explicit_api_key(self, sample_providers):
        """Test resolving explicitly provided API key"""
        manager = ProviderManager(sample_providers)
        
        api_key = manager.resolve_api_key(api_key="explicit-key")
        assert api_key == "explicit-key"
    
    @patch.dict(os.environ, {"LLM_API_KEY": "unified-key"}, clear=True)
    def test_resolve_unified_env_var(self, sample_providers):
        """Test resolving from unified env variable"""
        manager = ProviderManager(sample_providers)
        
        api_key = manager.resolve_api_key()
        assert api_key == "unified-key"
    
    @patch.dict(os.environ, {"DASHSCOPE_API_KEY": "dashscope-key"}, clear=True)
    def test_resolve_provider_specific_env_var(self, sample_providers):
        """Test resolving from provider-specific env variable"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        api_key = manager.resolve_api_key()
        assert api_key == "dashscope-key"
    
    @patch.dict(os.environ, {
        "LLM_API_KEY": "unified-key",
        "DASHSCOPE_API_KEY": "dashscope-key"
    }, clear=True)
    def test_resolve_priority_order(self, sample_providers):
        """Test API key resolution priority (unified > specific)"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        # Unified env var has priority
        api_key = manager.resolve_api_key()
        assert api_key == "unified-key"
        
        # Explicit key has highest priority
        api_key = manager.resolve_api_key(api_key="explicit-key")
        assert api_key == "explicit-key"
    
    @patch.dict(os.environ, {}, clear=True)
    def test_resolve_no_api_key_raises_error(self, sample_providers):
        """Test that missing API key raises error"""
        manager = ProviderManager(sample_providers)
        
        with pytest.raises(ValueError, match="API key not found"):
            manager.resolve_api_key()


class TestClientInitialization:
    """Test OpenAI client initialization"""
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_initialize_client_with_defaults(self, mock_openai, sample_providers):
        """Test initializing client with default configuration"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        client = manager.initialize_client()
        
        # Check OpenAI was called with correct params
        mock_openai.assert_called_once()
        call_kwargs = mock_openai.call_args[1]
        assert call_kwargs["api_key"] == "test-key"
        assert call_kwargs["base_url"] == "https://dashscope.aliyuncs.com/compatible-mode/v1"
        
        # Check manager state
        assert manager.model == "qwen-plus"
        assert manager.base_url == "https://dashscope.aliyuncs.com/compatible-mode/v1"
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    def test_initialize_client_with_custom_params(self, mock_openai, sample_providers):
        """Test initializing client with custom parameters"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        client = manager.initialize_client(
            api_key="custom-key",
            base_url="https://custom.com/v1",
            model="custom-model"
        )
        
        call_kwargs = mock_openai.call_args[1]
        assert call_kwargs["api_key"] == "custom-key"
        assert call_kwargs["base_url"] == "https://custom.com/v1"
        assert manager.model == "custom-model"
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_initialize_different_provider(self, mock_openai, sample_providers):
        """Test initializing client for different provider"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        client = manager.initialize_client(provider_id="deepseek")
        
        call_kwargs = mock_openai.call_args[1]
        assert call_kwargs["base_url"] == "https://api.deepseek.com"
        assert manager.model == "deepseek-chat"


class TestGetters:
    """Test getter methods"""
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_get_client(self, mock_openai, sample_providers):
        """Test getting initialized client"""
        manager = ProviderManager(sample_providers)
        
        assert manager.get_client() is None
        
        manager.initialize_client()
        assert manager.get_client() is not None
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_get_model(self, mock_openai, sample_providers):
        """Test getting current model"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        assert manager.get_model() is None
        
        manager.initialize_client()
        assert manager.get_model() == "qwen-plus"
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_get_base_url(self, mock_openai, sample_providers):
        """Test getting current base URL"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        assert manager.get_base_url() is None
        
        manager.initialize_client()
        assert manager.get_base_url() == "https://dashscope.aliyuncs.com/compatible-mode/v1"
    
    def test_get_current_provider(self, sample_providers):
        """Test getting current provider ID"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        assert manager.get_current_provider() == "qwen"


class TestFallbackLogic:
    """Test fallback provider functionality"""
    
    def test_get_fallback_provider_qwen_to_deepseek(self, sample_providers):
        """Test fallback from qwen to deepseek"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        fallback = manager.get_fallback_provider()
        assert fallback == "deepseek"
    
    def test_get_fallback_provider_deepseek_to_qwen(self, sample_providers):
        """Test fallback from deepseek to qwen"""
        manager = ProviderManager(sample_providers, default_provider="deepseek")
        
        fallback = manager.get_fallback_provider()
        assert fallback == "qwen"
    
    def test_get_fallback_provider_no_fallback(self, sample_providers):
        """Test fallback for provider without fallback"""
        manager = ProviderManager(sample_providers, default_provider="openai")
        
        fallback = manager.get_fallback_provider()
        assert fallback is None
    
    def test_get_fallback_provider_missing_deepseek(self):
        """Test fallback when deepseek not available"""
        providers = {
            "qwen": ProviderConfig(
                name="Qwen",
                base_url="https://dashscope.aliyuncs.com/v1",
                model="qwen-plus",
                env_var="LLM_API_KEY",
                fallback_env="DASHSCOPE_API_KEY"
            )
        }
        manager = ProviderManager(providers, default_provider="qwen")
        
        fallback = manager.get_fallback_provider()
        assert fallback is None


class TestTryFallback:
    """Test try_fallback method"""
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_try_fallback_success(self, mock_openai, sample_providers):
        """Test successful fallback to another provider"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        result = manager.try_fallback()
        
        assert result is True
        assert manager.current_provider == "deepseek"
        assert manager.model == "deepseek-chat"
    
    def test_try_fallback_disabled(self, sample_providers):
        """Test fallback when disabled"""
        manager = ProviderManager(
            sample_providers,
            default_provider="qwen",
            enable_fallback=False
        )
        
        result = manager.try_fallback()
        
        assert result is False
        assert manager.current_provider == "qwen"
    
    def test_try_fallback_no_fallback_available(self, sample_providers):
        """Test fallback when no fallback provider available"""
        manager = ProviderManager(sample_providers, default_provider="openai")
        
        result = manager.try_fallback()
        
        assert result is False
        assert manager.current_provider == "openai"
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {}, clear=True)
    def test_try_fallback_initialization_fails(self, mock_openai, sample_providers):
        """Test fallback when initialization fails (restores old provider)"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        result = manager.try_fallback()
        
        # Should fail (no API key), and restore old provider
        assert result is False
        assert manager.current_provider == "qwen"
    
    @patch('rob_box_voice.llm.provider_manager.OpenAI')
    @patch.dict(os.environ, {"LLM_API_KEY": "test-key"}, clear=True)
    def test_try_fallback_with_custom_params(self, mock_openai, sample_providers):
        """Test fallback with custom parameters"""
        manager = ProviderManager(sample_providers, default_provider="qwen")
        
        result = manager.try_fallback(
            api_key="fallback-key",
            model="custom-fallback-model"
        )
        
        assert result is True
        assert manager.model == "custom-fallback-model"
