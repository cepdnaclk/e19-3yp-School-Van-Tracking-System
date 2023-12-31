import { StyleSheet, View, Text, Image, TextInput, SafeAreaView, TouchableOpacity, KeyboardAvoidingView, Keyboard} from 'react-native'
import React, { useState, useEffect } from 'react'
import colors from '../constants/colors';

import {Ionicons, MaterialIcons} from '@expo/vector-icons'

const Login = ({navigation}) => {

    // Tracking Password visibility..
    const [isPasswordShown, SetPasswordShown] = useState(false);


    return (
    <SafeAreaView style = {styles.safearea}>
        <View style ={styles.container}>

            <View style={{marginVertical: 50}}>
                <Text style={styles.heading}><Ionicons name='location-outline' color={colors.red} size={25}/>Sure<Text>Way..</Text></Text>
            </View>
            <View style={{alignItems:'center'}}>
                <Image
                    source={require('../assets/SureWayLogo.png')}
                    style = {styles.logo}
                />

                <Text style={styles.login}>LOGIN</Text>
            </View>
            
            <View style={styles.inputBox}>
                <Ionicons name='person-circle-outline' size={20} style={styles.icons}/>
                <TextInput
                    placeholder='Username'
                    style = {styles.textInput}
                />
            </View>

            <View style={styles.inputBox}>
                <Ionicons name='lock-closed-outline' size={20} style={styles.icons}/>
                <TextInput
                    placeholder='Password'
                    secureTextEntry = {!isPasswordShown}
                    style = {styles.textInput}
                />

                <TouchableOpacity 
                    onPress={()=>SetPasswordShown(!isPasswordShown)}    // set the inputted password field to be hided..
                    style={{position: 'absolute', right: 15}}
                >
                    {
                        isPasswordShown == true ? (
                            <Ionicons name='eye-off' size={20} style={{color: colors.black}}/>
                        ): 
                        <Ionicons name='eye' size={20} style={{color: colors.black}}/>
                    }
                    
                </TouchableOpacity>
            </View>

            <TouchableOpacity style={styles.submitButton}>
                <Text style={{fontSize: 20, color:colors.black, fontFamily: 'NotoSansMono-Bold'}}>Login</Text>
            </TouchableOpacity>
            
            <TouchableOpacity>
                <Text style={{fontSize: 13, textDecorationLine: 'underline'}}>Forgot Password ?</Text>
            </TouchableOpacity>

            
            <View style={styles.signupPrompt}>
                <Text style={styles.signupText}>Not Registered ?</Text>
                <TouchableOpacity
                    onPress={()=>navigation.navigate("SignUp")}
                >
                    <Text style={styles.signup}>Create account</Text>
                </TouchableOpacity>
            </View>
            
        </View>
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({

    safearea: {
        flex: 1,
        backgroundColor: colors.white
    },

    container: {
        flex: 1,
        marginHorizontal: 22,
    },

    heading: {
        fontFamily: 'Acme-Regular',
        fontSize: 25,
        color: colors.black
    },

    logo: {
        height: 180,
        width: 300,
        resizeMode: 'contain'
    },

    login: {
        fontFamily: 'ReemKufi-Bold',
        fontSize: 45
    },

    inputBox: {
        flexDirection: 'row',
        alignItems: 'center',
        height: 50,
        width: '100%',
        backgroundColor: '#fff6e6',
        borderRadius: 50, // Adjust the value as needed
        padding: 8,
        marginBottom: 20,
    },
    
    textInput: {
        width: '100%',
        color: colors.gray,
        marginHorizontal: 5
    },

    icons: {
        color: colors.orange,
        marginLeft: 8
    },

    submitButton: {
        height: 50,
        width: '100%',
        borderRadius: 50, // Adjust the value as needed
        padding: 8,
        backgroundColor: colors.orange,
        marginVertical: 20,
        alignItems: 'center',
        justifyContent: 'center'
    },

    signupPrompt: {
        flexDirection: 'row',
        alignSelf: 'center',
        marginTop: 100
    },

    signupText:{
        fontFamily:'Ramabhadra-Regular'
    },

    signup:{
        marginLeft: 5,
        fontFamily:'Ramabhadra-Regular',
        color: colors.orange,
    }

});

export default Login