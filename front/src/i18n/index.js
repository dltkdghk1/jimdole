import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import en from './en.json';
import ko from './ko.json';

const resources = {
  en: { translation: en },
  ko: { translation: ko },
};

i18n.use(initReactI18next).init({
  resources,
  lng: 'ko', // 기본 언어 설정 (한국어)
  fallbackLng: 'en', // 언어가 없을 경우 영어로 대체
  interpolation: {
    escapeValue: false, // React는 XSS 방지를 위해 기본적으로 escape 처리함.
  },
});

export default i18n;
