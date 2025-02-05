#ifndef GyverOLEDMenu_h
#define GyverOLEDMenu_h

#define GM_N_INT(x) (__extension__({static const int __m_d_i = (x); &__m_d_i;}))
#define GM_N_U_INT(x) (__extension__({static const unsigned int __m_d_u_i = (x); &__m_d_u_i;}))
#define GM_N_FLOAT(x) (__extension__({static const float __m_d_f = (x); &__m_d_f;}))
#define GM_N_DOUBLE(x) (__extension__({static const double __m_d_d = (x); &__m_d_d;}))
#define GM_N_BYTE(x) (__extension__({static const byte __m_d_b = (x); &__m_d_b;}))

#define VAL_ACTION 0
#define VAL_INTEGER 1
#define VAL_FLOAT 2
#define VAL_DOUBLE 3
#define VAL_BYTE 4
#define VAL_BOOLEAN 5
#define VAL_U_INTEGER 6

#define MENU_IP_PRINT 0
#define MENU_IP_INC 1
#define MENU_IP_DEC 2

#ifndef MENU_SELECTED_H
#define MENU_SELECTED_H 10
#endif

#define MENU_ITEM_SELECT_W 127

#ifndef MENU_PARAMS_LEFT_OFFSET
#define MENU_PARAMS_LEFT_OFFSET 92
#endif

#define MENU_ITEM_PADDING_LEFT 2
#define MENU_ITEM_PADDING_TOP 2

#ifndef MENU_PAGE_ITEMS_COUNT
#define MENU_PAGE_ITEMS_COUNT 6
#endif

#ifndef MENU_FAST_K
#define MENU_FAST_K 4
#endif


typedef void (*cbOnChange)(const int index, const void* val, const byte valType);
typedef boolean (*cbOnPrintOverride)(const int index, const void* val, const byte valType);

const char* MENU_BOOLEAN_TEXT[]  = { "Off", "On" };

template< typename TGyverOLED >
class OledMenuItem {
public:
  boolean isSelect = false;
  boolean isChange = false;

  OledMenuItem() {}

  void setParams(char* str, uint8_t inc, uint8_t* val, const byte valType) {
    _str = str;
    _valType = valType;
    _inc = inc;
    _val = val;
  }

  void setPosition(const int x, const int y) {
    _x = x;
    _y = y;
    _y1 = _y + MENU_SELECTED_H;
    _text_y = _y + MENU_ITEM_PADDING_TOP;
  }

  void setOledInstance(TGyverOLED* oled, const int index) {
    _index = index;
    _oled = oled;
  }

  void drawItem(const boolean update = false) {
    _oled->rect(_x, _y, MENU_ITEM_SELECT_W, _y1, (isSelect && !isChange) ? OLED_FILL : OLED_CLEAR);

    if (isChange) {
      _oled->roundRect(MENU_PARAMS_LEFT_OFFSET - 4, _y, MENU_ITEM_SELECT_W, _y1, OLED_STROKE);
    }

    _oled->textMode((isSelect && !isChange) ? BUF_SUBTRACT : BUF_ADD);

    _oled->setCursorXY(_x + MENU_ITEM_PADDING_LEFT, _text_y);

    _oled->print((char *)_str);


    if (_valType != VAL_ACTION) {
      _oled->setCursorXY(MENU_PARAMS_LEFT_OFFSET, _text_y);

      switch (_valType) {
        case VAL_INTEGER:
          internalPrint(MENU_IP_PRINT);
          break;
      }
    }
    if (update) {
      _oled->update();
    }
  }

  void unselect(const boolean update = false) {
    isSelect = false;

    drawItem(update);
  }

  void select(const boolean update = false) {
    isSelect = true;

    drawItem(update);
  }

  void toggleChange() {
    if (_valType == VAL_ACTION) {
      callCb();

      return;
    }

    isChange = !isChange;

    drawItem(true);

    if (!cbImmediate && !isChange) {
      callCb();
    }
  }

  void setMinMax(uint8_t* min, uint8_t* max, int8_t* dp, uint8_t* symb) {
    _min = min;
    _max = max;
    _dp  = dp;
    _symb = symb;
  }

  void increment(const boolean isFast = false) {
    if (_valType == VAL_ACTION) {
      return;
    }

    prepareValUpdate();

    switch (_valType) {
      case VAL_INTEGER:
        internalPrint(MENU_IP_INC, isFast);
        break;
    }

    _oled->update();

    if (cbImmediate) {
      callCb();
    }
  }

  void decrement(const boolean isFast = false) {
    if (_valType == VAL_ACTION) {
      return;
    }

    prepareValUpdate();

    switch (_valType) {
      case VAL_INTEGER:
        internalPrint(MENU_IP_DEC, isFast);
        break;

    }

    _oled->update();

    if (cbImmediate) {
      callCb();
    }
  }


  void internalPrint(const byte mode, const boolean isFast = false) {
    uint8_t nextVal = 0;
    uint8_t __val = 0;
    int8_t __dp = 0;
    uint8_t __symb = 0;
    uint8_t __dec,__units = 0;
    int int_val = 0;

    if (mode != MENU_IP_PRINT) {

      
      if (mode == MENU_IP_INC) {
        nextVal = (uint8_t)*_val + (isFast ? ((_inc) * MENU_FAST_K) : _inc);
      } else if (mode == MENU_IP_DEC) {
        nextVal = (uint8_t)*_val - (isFast ? ((_inc) * MENU_FAST_K) : _inc);
      }

      if (nextVal > *_max) {
        *_val = *_min;
      } else if (nextVal < *_min) {
        *_val = *_max;
      } else {
        *_val = nextVal;
      }
      
    }

    if (!callPrintOverride()) {

      __dp = *_dp;
      __val = * _val;
      __symb = *_symb;

      if (__dp == 0)    // normal parameter printing 
        _oled->print(__val);

      
      if (__dp == 1){   // format X.X
        
        int __dec = __val / 10;
        _oled->print(__dec);
        _oled->print(".");

    
        int __units = __val % 10;
        _oled->print(__units);
      } 

      if (__dp == -1){   //format X * 10 
        int_val = __val * 10;
        _oled->print(int_val);
      }

      switch(__symb){
        case 0: break;
        case 1: _oled->print(" c");
                break;
        case 2: _oled->print(" %");
                break; 
        case 3: _oled->print(" A");
                break;
        case 4: _oled->print(" s");
                break;
        case 5: _oled->print(" mc");
                break;
        case 6: _oled->print(" ms");
                break;
        case 7: drawIcon8x8(0); 
                _oled->print("C");
                break;
        case 8: _oled->print(" kph");
                break;
      }

      
    }
  }

  void printBoolean(const byte mode = MENU_IP_PRINT) {
    if (mode != MENU_IP_PRINT) {
      *(boolean*)_val = !*(boolean*)_val;
    }

    if (!callPrintOverride()) {
      _oled->print(MENU_BOOLEAN_TEXT[*(boolean*)_val]);
    }
  }

  void onChange(cbOnChange cb, const boolean immediate = false) {
    _onChange = cb;
    cbImmediate = immediate;
  }

  void onPrintOverride(cbOnPrintOverride cb) {
    _onPrintOverride = cb;
  }

  void callCb() {
    if (_onChange == nullptr) {
      return;
    }

    _onChange(_index, _val, _valType);
  }

  boolean callPrintOverride() {
    if (_onPrintOverride == nullptr) {
      return false;
    }

    return _onPrintOverride(_index, _val, _valType);
  }

private:
  TGyverOLED* _oled = nullptr;
  int _index = 0;
  char * _str = nullptr;
  int _x;
  int _y;
  int _y1;
  int _text_y;
  byte _valType;
  uint8_t _inc = 1;
  uint8_t* _min = nullptr;
  uint8_t* _max = nullptr;
  int8_t* _dp = nullptr;
  uint8_t* _val = nullptr;
  uint8_t* _symb = nullptr;

  cbOnChange _onChange = nullptr;
  cbOnPrintOverride _onPrintOverride = nullptr;
  boolean cbImmediate = false;

  void prepareValUpdate() {
    _oled->rect(MENU_PARAMS_LEFT_OFFSET - 4, _y, MENU_ITEM_SELECT_W, _y1, OLED_CLEAR);
    _oled->roundRect(MENU_PARAMS_LEFT_OFFSET - 4, _y, MENU_ITEM_SELECT_W, _y1, OLED_STROKE);

    _oled->textMode(BUF_ADD);
    _oled->setCursorXY(MENU_PARAMS_LEFT_OFFSET, _text_y);
  }
};

// =================================================================================================================================


template< uint16_t _MS_SIZE, typename TGyverOLED >
class OledMenu {
public:
  byte currentPage = 1;
  boolean isMenuShowing = false;
  boolean cbImmediate = false;

  OledMenu(const TGyverOLED* oled): _oled((TGyverOLED*)oled)  {}

  // val

  void addItem(char * str) {
    doAddItem(str, VAL_ACTION, 0, nullptr, nullptr, nullptr,nullptr,nullptr);
  }

  void addItem(char * str, uint8_t inc, uint8_t* val, uint8_t* min, uint8_t* max, int8_t* dp, uint8_t* symb) {
    doAddItem(str, VAL_INTEGER, inc, val, min, max, dp, symb);
  }


  void selectNext(const boolean isFast = false) {
    if (!isMenuShowing) {
      return;
    }

    int selectedIdx = getSelectedItemIndex();

    if (selectedIdx == -1) {
      return;
    }

    if (oledMenuItems[selectedIdx].isChange) {
      oledMenuItems[selectedIdx].increment(isFast);
      return;
    }

    gotoIndex(selectedIdx, selectedIdx + 1);
  }

  void gotoIndex_(int selectTo)
  {
    if (!isMenuShowing) {
      return;
    }
    int selectedIdx = getSelectedItemIndex();

    if (selectedIdx == -1) {
      return;
    }
     gotoIndex(selectedIdx, selectTo);
  }


  void selectPrev(const boolean isFast = false) {
    if (!isMenuShowing) {
      return;
    }

    int selectedIdx = getSelectedItemIndex();

    if (selectedIdx == -1) {
      return;
    }

    if (oledMenuItems[selectedIdx].isChange) {
      oledMenuItems[selectedIdx].decrement(isFast);
      return;
    }

    gotoIndex(selectedIdx, selectedIdx - 1);
  }

  void toggleChangeSelected() {
    if (!isMenuShowing) {
      return;
    }

    int selectedIdx = getSelectedItemIndex();

    if (selectedIdx == -1) {
      return;
    }

    oledMenuItems[selectedIdx].toggleChange();
  }

  void onChange(cbOnChange cb, const boolean immediate = false) {
    _onItemChange = cb;
    cbImmediate = immediate;
  }

  void onPrintOverride(cbOnPrintOverride cb) {
    _onItemPrintOverride = cb;
  }

  void showMenu(const boolean val, const boolean update = true) {
    if (val == isMenuShowing) {
      return;
    }

    isMenuShowing = val;

    if (isMenuShowing) {
      renderPage(currentPage);
    } else {
      _oled->clear();
      setDefaultOledParams();

      if (update) {
        _oled->update();
      }
    }
  }

  void refresh() {
    renderPage(currentPage);
  }

  byte pageCount() {
    return getPageByIndex(_MS_SIZE - 1);
  }
  
  int getSelectedItemIndex() {
    for (int i = 0; i < _MS_SIZE; i++) {
      if (oledMenuItems[i].isSelect) {
        return i;
      }
    }

    return -1;
  }
  OledMenuItem<TGyverOLED> oledMenuItems[_MS_SIZE];

private:
  TGyverOLED* _oled = nullptr;
  int initInterator = 0;
  cbOnChange _onItemChange = nullptr;
  cbOnPrintOverride _onItemPrintOverride = nullptr;



  int getChangeableItemIndex() {
    for (int i = 0; i < _MS_SIZE; i++) {
      if (oledMenuItems[i].isChange) {
        return i;
      }
    }

    return -1;
  }

  void doAddItem(char* str, byte valType, uint8_t inc, uint8_t* val, uint8_t* min, uint8_t* max, int8_t* dp, uint8_t* symb) {
    if (!(initInterator < _MS_SIZE)) {
      return;
    }

    oledMenuItems[initInterator].setOledInstance(_oled, initInterator);
    oledMenuItems[initInterator].setParams(str, inc, val, valType);
    oledMenuItems[initInterator].onChange(_onItemChange, cbImmediate);
    oledMenuItems[initInterator].onPrintOverride(_onItemPrintOverride);
    oledMenuItems[initInterator].setMinMax(min, max, dp, symb);

    initInterator++;
  }

  void renderPage(const byte page, const boolean firstSelect = true) {
    if (page < 1) {
      return;
    }

    _oled->clear();

    int maxInPage = page * MENU_PAGE_ITEMS_COUNT;
    int minInPage = maxInPage - MENU_PAGE_ITEMS_COUNT;

    maxInPage = maxInPage > _MS_SIZE ? _MS_SIZE : maxInPage;

    int ordinalInc = 0;

    boolean selectedExist = getSelectedItemIndex() != -1;

    for (int i = minInPage; i < maxInPage; i++) {
      oledMenuItems[i].setPosition(0, ordinalInc * 10);

      if (!selectedExist && (firstSelect && ordinalInc == 0)) {
        oledMenuItems[i].select();
      }

      oledMenuItems[i].drawItem();

      ordinalInc++;
    }

    if (!selectedExist && !firstSelect) {
      oledMenuItems[maxInPage - 1].select();
    }

    currentPage = page;

    _oled->update();
  }

  void gotoIndex(const byte selectedIdx, int nextIdx) {
    byte nextIndexPage = 0;
    boolean isFirstSelect = true;

    oledMenuItems[selectedIdx].unselect();

    if (nextIdx < 0) {
      nextIndexPage = pageCount();
      isFirstSelect = false;
    } else if (nextIdx > (_MS_SIZE - 1)) {
      nextIndexPage = 1;
      isFirstSelect = true;
    } else {
      nextIndexPage = getPageByIndex(nextIdx);
      isFirstSelect = nextIndexPage > currentPage;
    }

    if (nextIndexPage != currentPage) {
      renderPage(nextIndexPage, isFirstSelect);
      return;
    }

    if (nextIdx < 0) {
      nextIdx = _MS_SIZE - 1;
    } else if (nextIdx > (_MS_SIZE - 1)) {
      nextIdx = 0;
    }

    oledMenuItems[nextIdx].select(true);
  }

  byte getPageByIndex(const byte index) {
        // ((index + 1) + MENU_PAGE_ITEMS_COUNT - 1) - one(1) was cut
    return (index + MENU_PAGE_ITEMS_COUNT) / MENU_PAGE_ITEMS_COUNT;
  }

  void setDefaultOledParams() {
    _oled->textMode(BUF_REPLACE); // default in GyverOLED.h
  }
};

#endif
